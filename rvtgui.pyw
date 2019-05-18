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
import shlex
import string
import os

import ctypes
myappid = 'mycompany.myproduct.subproduct.version' # arbitrary string
ctypes.windll.shell32.SetCurrentProcessExplicitAppUserModelID(myappid)

class MainWindow(tkinter.Tk):

    def __init__(self, channels, delay=50):

        self.channels = channels
        # for i in range(0, 256):
        #     if i not in self.channels.values():
        #         self.channels["/channel-" + str(i)] = i;

        self.delay = delay
        self.buffer = list([])
        self.packets = list([])
        self.parsing_deque = deque([])
        self.arduino = serial.Serial()
        self.checksum_errors = 0

        if not os.path.exists("logs"):
            os.mkdir("logs")
        else:
            pass

        filename = "logs/LOG-" + str(datetime.utcnow()
            ).replace(" ", "-").replace(":", "-") + ".rvt"
        self.logfile = open(filename, 'wb')

        self.baudrates = baudrates = list([
            19200, 38400, 57600, 115200, 230400,
            460800, 500000, 576000, 921600
        ])

        self.last_time = time.time()
        self.last_bytes = len(self.buffer)

        tkinter.Tk.__init__(self)
        self.title("Ground Control")
        self.wm_iconbitmap("launch.ico")
        self.tk.call('wm', 'iconphoto', self._w,
        tkinter.PhotoImage(file='launch.ico'))
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
        command=lambda: PlotterWindow(self.packets, self.channels)
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
        self.send_button = tkinter.Button(commsFrame, text='Send',
        state=tkinter.DISABLED, width=10, command=lambda:
        self.send_command(channelSend.get(), textInputBox.get()))
        self.send_button.grid(row=2, column=2, padx=5, pady=5)
        ports = self.refresh_ports()
        portselect = ttk.Combobox(commsFrame, values=ports)
        if len(ports) > 0:
            portselect.current(0)
        portselect.grid(row=3, column=0, padx=5, pady=5)
        tkinter.Button(commsFrame, text='Update Ports', width=12,
        command=lambda: self.refresh_ports(portselect)
        ).grid(row=3, column=1, padx=5, pady=5)
        baudselect = ttk.Combobox(commsFrame, values=baudrates)
        baudselect.current(8)
        baudselect.grid(row=4, columnspan=3, padx=5, pady=5)
        openButton = tkinter.Button(commsFrame, text='Open', width=10, command=lambda:
        self.open_port(portselect.get(), baudselect.get(), openButton))
        openButton.grid(row=3, column=2, padx=5, pady=5)
        commsFrame.grid(padx=5, pady=5)

        unlockFrame = tkinter.Frame(main_frame)
        unlock3 = tkinter.BooleanVar()
        unlock2 = tkinter.BooleanVar()
        unlock1 = tkinter.BooleanVar()
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

        dataFrame = tkinter.Frame(main_frame)
        self.clock_display = tkinter.Label(dataFrame, text="Time",
        font=("Consolas", 8))
        self.clock_display.grid(row=0, columnspan=2, padx=5, pady=5)
        self.freq_display = tkinter.Label(dataFrame, text="Frequency",
        font=("Consolas", 8), width=10)
        self.freq_display.grid(row=0, column=2, padx=5, pady=5)
        self.bps_display = tkinter.Label(dataFrame, width=10, text="BPS",
        anchor=tkinter.E, justify=tkinter.RIGHT, font=("Consolas", 8))
        self.bps_display.grid(row=0, column=3, padx=5, pady=5)
        self.err_display = tkinter.Label(dataFrame, text="Errors",
        anchor=tkinter.E, justify=tkinter.RIGHT, font=("Consolas", 8))
        self.err_display.grid(row=1, columnspan=2, padx=5, pady=5)
        self.bytes_display = tkinter.Label(dataFrame, text="Bytes",
        anchor=tkinter.E, justify=tkinter.RIGHT, font=("Consolas", 8))
        self.bytes_display.grid(row=1, column=2, padx=5, pady=5)
        self.packets_ticker = tkinter.Label(dataFrame, text="Packets",
        anchor=tkinter.E, justify=tkinter.RIGHT, font=("Consolas", 8))
        self.packets_ticker.grid(row=1, column=3, padx=5, pady=5)
        dataFrame.grid(padx=5, pady=5)

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

        self.err_display["text"] = str(self.checksum_errors) + " checksum errors"
        self.bytes_display["text"] = str(len(self.buffer)) + " bytes"
        self.packets_ticker["text"] = str(len(self.packets)) + " packets"

        self.last_time = time.time()
        self.last_bytes = len(self.buffer)

        while self.arduino.is_open and self.arduino.inWaiting():
            byte = int.from_bytes(self.arduino.read(), byteorder='big')
            self.parsing_deque.append(byte)
            self.buffer.append(byte)
            if True:
                self.logfile.write(bytes([byte]))
                self.logfile.flush()

        if len(self.parsing_deque) > 0:
            (packets, err) = rvtr.parse(self.parsing_deque)
            self.checksum_errors += err
            for p in packets:
                self.packets.append(p)

    def open_port(self, port, baudrate, button=None):

        if self.arduino.is_open:
            self.arduino.reset_input_buffer()
            self.arduino.reset_output_buffer()
            self.arduino.close()
            print("Closed " + str(self.arduino) + ".")
            self.send_button["state"] = tkinter.DISABLED
        else:
            self.arduino.close()
            self.arduino.port = port
            self.arduino.baudrate = baudrate
            try:
                self.arduino.open()
                self.arduino.reset_input_buffer()
                self.arduino.reset_output_buffer()
            except Exception as e:
                print(e)
                return
            print("Opened " + str(self.arduino) + ".")
            print(self.send_button)
            self.send_button["state"] = tkinter.NORMAL
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
            data = rvtr.pack(text)
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
                self.send_command("/ground/motor-unlock", "0u8")
            else:
                self.lock_motor(unlock1, unlock2, unlock3,
                                checkbox1, checkbox2, checkbox3)
        elif stage == 2:
            if unlock2.get() == 1:
                checkbox3['state'] = tkinter.NORMAL
                self.send_command("/ground/motor-unlock", "1u8")
            else:
                self.lock_motor(unlock1, unlock2, unlock3,
                                checkbox1, checkbox2, checkbox3)
        elif stage == 3:
            if unlock3.get() == 1:
                self.send_command("/ground/motor-unlock", "2u8")
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
        self.send_command("/ground/motor-lock", "")

    def clear_buffers(self):

        self.buffer.clear()
        self.packets.clear()
        self.parsing_deque.clear()
        self.checksum_errors = 0


class HexdumpWindow(tkinter.Toplevel):

    def __init__(self, buffer, display_bytes=512, delay=100):

        self.display_bytes = display_bytes
        self.buffer = buffer
        self.delay = delay
        self.buffer_size = 0

        tkinter.Toplevel.__init__(self)
        self.title("Hexdump")
        self.tk.call('wm', 'iconphoto', self._w,
        tkinter.PhotoImage(file='launch.ico'))
        self.protocol("WM_DELETE_WINDOW", self.destroy)

        header = tkinter.Text(self, wrap="none",
        height=1, font=("Ubuntu Mono", 14), borderwidth=3, relief='flat')
        header.insert(tkinter.END, "       00 01 02 03 04 05 06 07   " +
            "08 09 0A 0B 0C 0D 0E 0F    ASCII")
        header.pack(fill=tkinter.X)
        self.textOutput = tkinter.Text(self, wrap="none", width = 28*3,
        height=27, font=("Ubuntu Mono", 14), borderwidth=3, relief='sunken')
        self.textOutput.pack(fill=tkinter.BOTH, expand=tkinter.YES)
        self.begin_loop()

    def begin_loop(self):

        self.update()
        self.after(self.delay, self.begin_loop)

    def update(self):

        if len(self.buffer) == self.buffer_size:
            return

        self.force_render = False
        self.buffer_size = len(self.buffer)
        start = len(self.buffer) - self.display_bytes;
        if start < 0:
            start = 0
        start = int(numpy.ceil(start/16.0)*16);
        total_string = ""
        hexdumpString = ""
        asciiString = ""
        column = 0
        row = int(start/16)
        toPrint = self.buffer[start:]
        self.textOutput.delete(1.0, tkinter.END)

        for i, x in enumerate(toPrint):
            if chr(x) in string.printable and x not in (10, 11, 12, 13):
                asciiString += chr(x)
            else:
                asciiString += "."
            if column % 16 == 0:
                hexdumpString += "{:04X}".format(row) + "   "
                row += 1
            hexdumpString += "{:02X}".format(x) + " ";
            column = (column + 1) % 16
            if column % 16 == 0 or i == len(toPrint) - 1:
                total_string += hexdumpString.ljust(60) + asciiString + "\n"
                hexdumpString = ""
                asciiString = ""
            elif column % 8 == 0:
                hexdumpString += "  "

        self.textOutput.insert(tkinter.END, total_string)
        self.textOutput.see(tkinter.END)


class PacketWindow(tkinter.Toplevel):

    def __init__(self, packets, channels, display_packets=100, delay=100):

        self.packets = packets
        self.channels = channels
        self.display_packets = display_packets
        self.delay = delay
        self.num_packets = 0
        self.print_mode = False

        tkinter.Toplevel.__init__(self)
        self.title("Packet Viewer")
        self.tk.call('wm', 'iconphoto', self._w,
        tkinter.PhotoImage(file='launch.ico'))
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

        self.ascii_switch = tkinter.BooleanVar()
        self.ascii_switch.set(True)
        tkinter.Checkbutton(topFrame, var=self.ascii_switch, text="ASCII"
        ).grid(row=0, column=2, padx=5, pady=5)

        topFrame.pack()

        self.output = tkinter.Text(self, width = 25*3, wrap="none",
        font=("Ubuntu Mono", 14), borderwidth=3, relief='sunken')
        self.output.pack(fill=tkinter.BOTH, expand=tkinter.YES)

        self.begin_loop()

    def begin_loop(self):

        self.update()
        self.after(self.delay, self.begin_loop)

    def update(self):

        if self.num_packets == len(self.packets) and self.print_mode == self.ascii_switch.get():
            return

        self.print_mode = self.ascii_switch.get()
        self.num_packets = len(self.packets)
        self.output.delete(1.0, tkinter.END)
        output = ""
        index = 0
        for p in self.packets:
            id = p[3]
            if (self.channelFilter.get() == "All Channels") or (
            self.channels[self.channelFilter.get()] == id):
                output += "{:04X}".format(index) + "   "
                channel = list(self.channels.keys())[
                list(self.channels.values()).index(id)]
                output += channel.ljust(len(channel) + 2)
                if self.ascii_switch.get():
                    for x in p[4:-2]:
                        if chr(x) in string.printable and x not in (
                        10, 11, 12, 13):
                            output += chr(x)
                        else:
                            output += "."
                    output += "\n"
                else:
                    output += " ".join("{:02X}".format(x) for x in p) + "\n"
            index += 1
        self.output.insert(tkinter.END, output)
        self.output.see(tkinter.END)


class PlotterWindow(tkinter.Toplevel):

    def __init__(self, packets, channels, title="Raw Incoming Bytes", plot_recent=100, delay=25):

        self.packets = packets
        self.channels = channels
        self.plot_recent = plot_recent
        self.delay = delay

        tkinter.Toplevel.__init__(self)
        self.title(title)
        self.tk.call('wm', 'iconphoto', self._w,
        tkinter.PhotoImage(file='launch.ico'))
        self.protocol("WM_DELETE_WINDOW", self.destroy)

        topFrame = tkinter.Frame(self)
        bottomFrame = tkinter.Frame(self)

        tkinter.Label(topFrame, text="Enter parsing format: ",
        ).grid(row=0, column=0, padx=5, pady=5)
        tkinter.Label(topFrame, text="Filter for incoming channels: "
        ).grid(row=1, column=0, padx=5, pady=5)

        self.format_input = tkinter.Entry(topFrame, width=50)
        self.format_input.grid(row=0, column=1, padx=5, pady=5)

        channel_keys = list(self.channels.keys())
        channel_keys.sort(key = lambda x: self.channels[x])
        self.channel_select = ttk.Combobox(topFrame, width=30,
        values=list(channel_keys))
        self.channel_select.grid(row=1, column=1, padx=5, pady=5)
        topFrame.pack()

        figure = matplotlib.figure.Figure(dpi=100)
        self.plot = figure.add_subplot(111);

        self.canvas = FigureCanvasTkAgg(figure, bottomFrame)
        self.canvas.draw()
        self.canvas.get_tk_widget().pack(fill=tkinter.BOTH, expand=tkinter.YES)
        bottomFrame.pack(fill=tkinter.BOTH, expand=tkinter.YES)

        self.begin_loop()

    def begin_loop(self):

        self.update()
        self.after(self.delay, self.begin_loop)

    def update(self):

        format = self.format_input.get()
        num_channels = len(shlex.split(format))
        if not num_channels:
            return

        to_plot = self.packets[-self.plot_recent:]

        data_buffer = list([])
        legend = list([])

        for i in range(0, num_channels):
            data_buffer.append(list([]))
            legend.append("Channel " + str(i+1))

        for packet in to_plot:
            id = packet[3]
            channel = self.channel_select.get()
            if channel != '' and self.channels[channel] == id:
                data = packet[4:-2]
                parsed = rvtr.unpack(data, format)

                for i in range(0, num_channels):
                    if i < len(parsed):
                        data_buffer[i].append(parsed[i])
                    else:
                        data_buffer[i].append(numpy.nan)

        self.plot.clear()
        t = numpy.arange(0, len(data_buffer[0]), 1)
        for i in range(0, num_channels):
            color = 'k'
            if i % 3 == 1:
                color = 'b'
            if i % 3 == 2:
                color = 'r'
            self.plot.plot(t, data_buffer[i], color)
        self.plot.legend(legend, loc='upper right')
        self.plot.grid()
        self.canvas.draw()


if __name__ == "__main__":

    channels = {
        "/null": 0,
        "/ground/ping": 1,
        "/ground/echo-commands": 2,
        "/ground/echo-loglist": 3,
        "/ground/echo-logs": 4,
        "/ground/echo-channels": 5,
        "/ground/stop-log": 8,
        "/ground/begin-log": 9,
        "/ground/set-lock": 18,
        "/ground/small-talk": 34,
        "/rocket/console": 35,
        "/ground/echo-status": 47,
        "/ground/set-status": 48,
        "/ground/motor-unlock": 60,
        "/ground/motor-lock": 61,
        "/ground/fill-nitrous": 80,
        "/ground/disc-feedline": 81,
        "/ground/bleed-nitrous": 85,
        "/ground/echo-tests": 96,
        "/ground/perform-test": 97,
        "/ground/hardware-reboot": 124,
        "/ground/abort": 125,
        "/ground/reset": 126,
        "/ground/shutdown": 127,
        "/rocket/motor-info": 130,
        "/rocket/voltage": 131
    }

    print("Starting R@VT control.")

    MainWindow(channels).mainloop()