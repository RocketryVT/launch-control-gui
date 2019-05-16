import tkinter
from tkinter import ttk
import time
from datetime import datetime
import numpy
import matplotlib
import serial
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
from serial.tools import list_ports
from collections import deque
import rvtr


class MainWindow(tkinter.Tk):

    def __init__(self, channels, delay=100):

        self.channels = channels
        self.delay = delay

        self.buffer = list([])
        self.packets = list([])
        self.parsing_deque = deque([])
        self.arduino = serial.Serial()

        self.baudates = baudrates = list([
            19200, 38400, 57600, 115200, 230400,
            460800, 500000, 576000, 921600
        ])

        self.last_time = time.time()
        self.last_bytes = len(self.buffer)

        tkinter.Tk.__init__(self)
        self.title("Ground Control")
        self.tk.call('wm', 'iconphoto', self._w,
        tkinter.PhotoImage(file='glider.ico'))
        self.protocol("WM_DELETE_WINDOW", self.destroy)
        self.resizable(width=False, height=False)

        main_frame = tkinter.Frame(self, borderwidth=2, relief='groove')

        windowsFrame = tkinter.Frame(main_frame)
        tkinter.Button(windowsFrame, text='Hexdump', width=12,
        command=lambda: HexdumpWindow(self.buffer)
        ).grid(row=1, column=0, padx=5, pady=5)
        tkinter.Button(windowsFrame, text='Packet Viewer', width=12,
        command=lambda: PacketWindow(self.packets, self.channels)
        ).grid(row=1, column=1, padx=5, pady=5)
        tkinter.Button(windowsFrame, text='Plotter', width=12,
        command=lambda: PlotterWindow(self.buffer)
        ).grid(row=1, column=2, padx=2, pady=5)
        tkinter.Button(windowsFrame, text='Clear Buffers', width=12,
        command=self.clear_buffers).grid(row=2, column=1, padx=5, pady=5)
        windowsFrame.grid(padx=5, pady=5)

        commsFrame = tkinter.Frame(main_frame)
        channel_keys = list(channels.keys())
        channel_keys.sort(key = lambda x: channels[x])
        tkinter.Label(commsFrame, text="Send commands to the rocket " +
        "via this dialogue.").grid(row=0, columnspan=3)
        channelSend = ttk.Combobox(commsFrame,
        values=channel_keys, width=30)
        if len(channel_keys) > 0:
            channelSend.current(0)
        channelSend.grid(row=2, column=0, columnspan=2, padx=5, pady=5)
        textInputBox = tkinter.Entry(commsFrame, width=50)
        textInputBox.grid(row=1, columnspan=3)
        tkinter.Button(commsFrame, text='Send', width=10, command=lambda:
        self.send_command(channelSend.get(), textInputBox.get())
        ).grid(row=2, column=2, padx=5, pady=5)
        ports = self.refresh_ports()
        portselect = ttk.Combobox(commsFrame, values=ports)
        if len(ports) > 0:
            portselect.current(0)
        portselect.grid(row=3, column=0, padx=5, pady=5)
        tkinter.Button(commsFrame, text='Update Ports', width=12,
        command=lambda: self.refresh_ports(portselect)
        ).grid(row=3, column=1, padx=5, pady=5)
        baudselect = ttk.Combobox(commsFrame, values=baudrates)
        baudselect.current(3)
        baudselect.grid(row=4, columnspan=3, padx=5, pady=5)
        openButton = tkinter.Button(commsFrame, text='Open', width=10, command=lambda:
        self.open_port(portselect.get(), baudselect.get(), openButton))
        openButton.grid(row=3, column=2, padx=5, pady=5)
        commsFrame.grid(padx=5, pady=5)

        dataFrame = tkinter.Frame(main_frame)
        self.clock_display = tkinter.Label(dataFrame, text="Time",
        font=("Consolas", 10))
        self.clock_display.grid(row=1, columnspan=2, padx=5, pady=5)
        self.freq_display = tkinter.Label(dataFrame, text="Frequency",
        font=("Consolas", 10), width=10)
        self.freq_display.grid(row=1, column=2, padx=5, pady=5)
        self.bps_display = tkinter.Label(dataFrame, width=10, text="BPS",
        anchor=tkinter.E, justify=tkinter.RIGHT, font=("Consolas", 10))
        self.bps_display.grid(row=1, column=3, padx=5, pady=5)
        dataFrame.grid(padx=5, pady=5)

        unlockFrame = tkinter.Frame(main_frame)
        unlock3 = tkinter.IntVar()
        unlock2 = tkinter.IntVar()
        unlock1 = tkinter.IntVar()
        checkbox3 = tkinter.Checkbutton(unlockFrame, text='Unlock 3',
        state=tkinter.DISABLED, variable=unlock3,
        command=lambda:
        self.handle_motor_lock(unlock1, unlock2, unlock3,
        checkbox1, checkbox2, checkbox3, 3))
        checkbox3.grid(column=2, row=0, padx=5, pady=5)
        checkbox2 = tkinter.Checkbutton(unlockFrame, text='Unlock 2',
        state=tkinter.DISABLED, variable=unlock2,
        command=lambda:
        self.handle_motor_lock(unlock1, unlock2, unlock3,
        checkbox1, checkbox2, checkbox3, 2))
        checkbox2.grid(column=1, row=0, padx=5, pady=5)
        checkbox1 = tkinter.Checkbutton(unlockFrame,
        text='Unlock 1', variable=unlock1, command=lambda:
        self.handle_motor_lock(unlock1, unlock2, unlock3,
        checkbox1, checkbox2, checkbox3, 1))
        checkbox1.grid(column=0, row=0, padx=5, pady=5)
        unlockFrame.grid(padx=5, pady=5)

        main_frame.grid(padx=5, pady=5)

        self.begin_loop()

    def begin_loop(self):

        self.update()
        self.after(self.delay, self.begin_loop)

    def update(self):

        currTime = datetime.utcnow();
        self.clock_display["text"] = str(currTime) + " UTC"

        freq = numpy.round(100/(time.time() - self.last_time))/100.0
        self.freq_display["text"] = "{0:.2f}".format(freq) + " Hz"

        bps = ((len(self.buffer) - self.last_bytes)/(time.time() - self.last_time))
        self.bps_display["text"] = "{0:.1f}".format(numpy.round(bps*10)/10.0) + " B/s"

        self.last_time = time.time()
        self.last_bytes = len(self.buffer)

        while self.arduino.is_open and self.arduino.inWaiting():
            byte = int.from_bytes(self.arduino.read(), byteorder='big')
            self.parsing_deque.append(byte)
            self.buffer.append(byte)

        if len(self.parsing_deque) > 0:
            for p in rvtr.parse(self.parsing_deque):
                self.packets.append(p)

    def open_port(self, port, baudrate, button=None):

        if self.arduino.is_open:
            self.arduino.close()
            print("Closed " + str(self.arduino) + ".")
        else:
            self.arduino.close()
            self.arduino.port = port
            self.arduino.baudrate = baudrate
            try:
                self.arduino.open()
            except Exception as e:
                print(e)
                return
            print("Opened", port)
        if button is not None:
            if self.arduino.is_open:
                button["text"] = "Close"
            else:
                button["text"] = "Open"

    def refresh_ports(self, selector=None):

        ports = []
        devices = list_ports.comports(True)
        for x in devices:
            ports.append(x.device)
        if selector is not None:
            selector["values"] = ports
            if len(ports) == 0:
                selector.set("")
            else:
                selector.current(0)
        return ports

    def send_command(self, channel, text):

        if self.arduino.is_open:
            data = rvtr.packetify(text)
            id = self.channels[channel]
            packet = rvtr.buildPacket(id, data)
            self.arduino.write(bytearray(packet))
        else:
            print("Cannot send; port not open")

    def handle_motor_lock(self, unlock1, unlock2, unlock3,
                          checkbox1, checkbox2, checkbox3, stage):
        if stage == 1:
            if unlock1.get() == 1:
                checkbox2['state'] = tkinter.NORMAL
                self.send_command("/rocket/motor-unlock", "0u8")
            else:
                self.lock_motor(unlock1, unlock2, unlock3,
                                checkbox1, checkbox2, checkbox3)
        elif stage == 2:
            if unlock2.get() == 1:
                checkbox3['state'] = tkinter.NORMAL
                self.send_command("/rocket/motor-unlock", "1u8")
            else:
                self.lock_motor(unlock1, unlock2, unlock3,
                                checkbox1, checkbox2, checkbox3)
        elif stage == 3:
            if unlock3.get() == 1:
                self.send_command("/rocket/motor-unlock", "2u8")
            else:
                self.lock_motor(unlock1, unlock2, unlock3,
                                checkbox1, checkbox2, checkbox3)
        else:
            print("Invalid stage: " + str(stage))

    def lock_motor(self, unlock1, unlock2, unlock3,
                   checkbox1, checkbox2, checkbox3):

        checkbox1.deselect()
        checkbox2.deselect()
        checkbox3.deselect()
        checkbox1['state'] = tkinter.NORMAL
        checkbox2['state'] = tkinter.DISABLED
        checkbox3['state'] = tkinter.DISABLED
        self.send_command("/rocket/motor-lock", "")

    def clear_buffers(self):

        self.buffer.clear()
        self.packets.clear()
        self.parsing_deque.clear()


class HexdumpWindow(tkinter.Toplevel):

    def __init__(self, buffer, display_bytes=512, delay=100):

        self.display_bytes = display_bytes
        self.buffer = buffer
        self.delay = delay

        tkinter.Toplevel.__init__(self)
        self.title("Hexdump")
        self.tk.call('wm', 'iconphoto', self._w,
        tkinter.PhotoImage(file='glider.ico'))
        self.protocol("WM_DELETE_WINDOW", self.destroy)
        self.textOutput = tkinter.Text(self, width = 20*3,
        font=("Consolas Bold", 16), borderwidth=3, relief='sunken')
        self.textOutput.pack(fill=tkinter.BOTH, expand=tkinter.YES)

        self.begin_loop()

    def begin_loop(self):

        self.update()
        self.after(self.delay, self.begin_loop)

    def update(self):

        self.textOutput.delete(1.0, tkinter.END)
        start = len(self.buffer) - self.display_bytes;
        if start < 0:
            start = 0
        start = int(numpy.ceil(start/16.0)*16);
        hexdumpString = ""
        i = 0
        row = int(start/16)
        toPrint = self.buffer[start:]
        for x in toPrint:
            if i % 16 == 0:
                hexdumpString += "{:04X}".format(row) + "   "
                row += 1
            hexdumpString += "{:02X}".format(x) + " ";
            i += 1
            if i % 16 == 0:
                hexdumpString += "\n"
            elif i % 8 == 0:
                hexdumpString += "  "

        self.textOutput.insert(tkinter.END, hexdumpString)
        self.textOutput.see(tkinter.END)


class PacketWindow(tkinter.Toplevel):

    def __init__(self, packets, channels, display_packets=100, delay=100):

        self.packets = packets
        self.channels = channels
        self.display_packets = display_packets
        self.delay = delay
        self.max_channel_length = max([len(x) for x in channels])

        tkinter.Toplevel.__init__(self)
        self.title("Packet Viewer")
        self.tk.call('wm', 'iconphoto', self._w,
        tkinter.PhotoImage(file='glider.ico'))
        self.protocol("WM_DELETE_WINDOW", self.destroy)

        topFrame = tkinter.Frame(self)
        bottomFrame = tkinter.Frame(self)

        tkinter.Label(topFrame, text="Filter for incoming channels: "
        ).grid(row=0, column=0, padx=5, pady=5)

        channel_keys = list(channels.keys())
        channel_keys.sort(key = lambda x: channels[x])
        self.channelFilter = ttk.Combobox(topFrame, width=30,
        values=list(['All Channels']) + list(channel_keys))
        self.channelFilter.grid(row=0, column=1, padx=5, pady=5)
        self.channelFilter.current(0)
        topFrame.pack()

        self.output = tkinter.Text(self, width = 25*3,
        font=("Consolas Bold", 16), borderwidth=3, relief='sunken')
        self.output.pack(fill=tkinter.BOTH, expand=tkinter.YES)

        self.begin_loop()

    def begin_loop(self):

        self.update()
        self.after(self.delay, self.begin_loop)

    def update(self):

        self.output.delete(1.0, tkinter.END)
        output = ""
        index = 0
        for p in self.packets:
            id = p[3]
            if (self.channelFilter.get() == "All Channels") or (self.channels[self.channelFilter.get()] == id):
                output += "{:04X}".format(index) + "   "
                channel = list(self.channels.keys())[list(self.channels.values()).index(id)]
                output += channel.ljust(self.max_channel_length + 2)
                output += " ".join("{:02X}".format(x) for x in p) + "\n"
            index += 1
        self.output.insert(tkinter.END, output)
        self.output.see(tkinter.END)


class PlotterWindow(tkinter.Toplevel):

    def __init__(self, buffer, title="Raw Incoming Bytes", plot_bytes=100, delay=1000):

        self.buffer = buffer
        self.plot_bytes = plot_bytes
        self.delay = delay

        tkinter.Toplevel.__init__(self)
        self.title(title)
        self.tk.call('wm', 'iconphoto', self._w,
        tkinter.PhotoImage(file='glider.ico'))
        self.protocol("WM_DELETE_WINDOW", self.destroy)

        figure = matplotlib.figure.Figure(dpi=100)
        self.plot = figure.add_subplot(111);

        self.canvas = FigureCanvasTkAgg(figure, self)
        self.canvas.draw()
        self.canvas.get_tk_widget().pack(fill=tkinter.BOTH, expand=tkinter.YES)

        self.begin_loop()

    def begin_loop(self):

        self.update()
        self.after(self.delay, self.begin_loop)

    def update(self):

        self.plot.clear()
        toPlot = self.buffer[-self.plot_bytes:]
        elements = len(toPlot)
        t = numpy.arange(len(self.buffer) - elements, len(self.buffer), 1)
        self.plot.plot(t, toPlot, 'k')
        self.canvas.draw()


if __name__ == "__main__":

    channels = {
        "/null": 0,
        "/ground/ping": 1,
        "/rocket/ping": 2,
        "/relay/ping": 3,
        "/rocket/console": 4,
        "/relay/rocket/console": 5,
        "/relay/console": 6,
        "/rocket/status": 7,
        "/relay/rocket/status": 8,
        "/ground/fill-nitrous": 9,
        "/ground/disconnect-feed-line": 10,
        "/rocket/imudata": 11,
        "/relay/rocket/imudata": 12,
        "/ground/all-stop": 13,
        "/ground/reset": 14,
        "/ground/perform-test": 15,
        "/ground/relay-mode": 16,
        "/relay/ground/relay-mode": 17,
        "/rocket/voltage": 18,
        "/rocket/motor-info": 19,
        "/rocket/motor-unlock": 20,
        "/rocket/motor-lock": 21
    }

    print("Starting R@VT control.")

    MainWindow(channels).mainloop()
