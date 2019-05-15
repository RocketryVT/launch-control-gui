import tkinter as tk
from tkinter import ttk
import matplotlib
import time
from datetime import datetime
import numpy as np
matplotlib.use("TKAgg")
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
from matplotlib.figure import Figure
import matplotlib.animation as animation
from matplotlib import style
style.use('ggplot')
import random
import serial
from serial.tools import list_ports
import rvtr
from collections import deque

def open_port(port, baudrate):
    if arduino.is_open:
        arduino.close()
        openButton["text"] = "Open";
        print("Closed " + str(arduino) + ".")
        return
    try:
        arduino.close()
        arduino.port = port
        arduino.baudrate = baudrate
        arduino.open()
        openButton["text"] = "Close";
        print("Successfully opened port " + port + " (" + str(baudrate) + ")")
    except Exception as e:
        print("Failed to open arduino serial port. Reason:\n" + str(e))
        arduino.close()
        openButton["text"] = "Open";

def open_hexdump():
    global hexdumpOpen
    global hexdump
    global hexdump_window
    try:
        hexdumpOpen
    except:
        hexdumpOpen = False
    if hexdumpOpen:
        return
    hexdumpOpen = True
    hexdump_window = tk.Toplevel()
    hexdump_window.title("Hexdump")
    hexdump_window.tk.call('wm', 'iconphoto', hexdump_window._w,
    window_icon)
    hexdump = tk.Text(hexdump_window, width = 20*3,
    font=("Consolas Bold", 16), borderwidth=3, relief='sunken')
    hexdump.pack(fill=tk.BOTH, expand=tk.YES)
    hexdump_window.protocol("WM_DELETE_WINDOW",
    lambda: close_hexdump(hexdump_window))
    hexdump_window.mainloop()

def close_hexdump(hexdump):
    global hexdumpOpen
    hexdumpOpen = False
    hexdump.destroy()

def open_plotter():
    global live_plot
    global canvas
    global plotterOpen
    try:
        plotterOpen
    except:
        plotterOpen = False
    if plotterOpen:
        return
    plotterOpen = True
    plotter = tk.Toplevel()
    plotter.title("Plotter")
    plotter.tk.call('wm', 'iconphoto', plotter._w,
    window_icon)
    plotter.protocol("WM_DELETE_WINDOW",
    lambda: close_plotter(plotter))

    f = Figure(dpi=100)
    t = np.arange(0, len(input_buffer), 1)
    live_plot = f.add_subplot(111);
    live_plot.plot(t, input_buffer)

    canvas = FigureCanvasTkAgg(f, plotter)
    canvas.draw()
    canvas.get_tk_widget().pack(fill=tk.BOTH, expand=tk.YES)

    plotter.mainloop()

def close_plotter(plotter):
    global live_plot
    global canvas
    global plotterOpen
    plotterOpen = False
    plotter.destroy()
    try:
        del live_plot
        del canvas
    except:
        pass

def open_packet_window():
    global packetOpen
    global packet_window
    global packetOutput
    global channelFilter

    try:
        packetOpen
    except:
        packetOpen = False
    if packetOpen:
        return
    packetOpen = True
    packet_window = tk.Toplevel()
    packet_window.title("Packet Viewer")
    packet_window.tk.call('wm', 'iconphoto', packet_window._w,
    window_icon)

    topFrame = tk.Frame(packet_window)
    bottomFrame = tk.Frame(packet_window)

    tk.Label(topFrame, text="Filter for incoming channels: "
    ).grid(row=0, column=0, padx=5, pady=5)

    channelFilter = ttk.Combobox(topFrame, width=30,
    value=list(['All Channels']) + channels)
    channelFilter.grid(row=0, column=1, padx=5, pady=5)
    channelFilter.current(0)
    topFrame.pack()

    packetOutput = tk.Text(packet_window, width = 25*3,
    font=("Consolas Bold", 16), borderwidth=3, relief='sunken')
    packetOutput.pack(fill=tk.BOTH, expand=tk.YES)
    packet_window.grid()

    packet_window.protocol("WM_DELETE_WINDOW",
    lambda: close_packet_window(packet_window))
    packet_window.mainloop()

def close_packet_window(packet_window):
    global packetOpen
    packetOpen = False
    packet_window.destroy()

def send_command(channel, text):
    global arduino
    try:
        data = rvtr.packetify(text)
        id = channelDict[channel]
        packet = rvtr.buildPacket(id, data)
        arduino.write(bytearray(packet))
    except Exception as e:
        print("Failed to send command: " + str(e))
        pass

def handle_motor_lock(unlock1, unlock2, unlock3,
checkbox1, checkbox2, checkbox3, stage):
    if stage == 1:
        if unlock1.get() == 1:
            checkbox2['state'] = tk.NORMAL
            send_command("/rocket/motor-unlock", "0u8")
        else:
            lock_motor(unlock1, unlock2, unlock3,
            checkbox1, checkbox2, checkbox3)
    elif stage == 2:
        if unlock2.get() == 1:
            checkbox3['state'] = tk.NORMAL
            send_command("/rocket/motor-unlock", "1u8")
        else:
            lock_motor(unlock1, unlock2, unlock3,
            checkbox1, checkbox2, checkbox3)
    elif stage == 3:
        if unlock3.get() == 1:
            send_command("/rocket/motor-unlock", "2u8")
        else:
            lock_motor(unlock1, unlock2, unlock3,
            checkbox1, checkbox2, checkbox3)
    else:
        print("Invalid stage: " + str(stage))

def lock_motor(unlock1, unlock2, unlock3,
checkbox1, checkbox2, checkbox3):
    checkbox1.deselect()
    checkbox2.deselect()
    checkbox3.deselect()
    checkbox1['state'] = tk.NORMAL
    checkbox2['state'] = tk.DISABLED
    checkbox3['state'] = tk.DISABLED
    send_command("/rocket/motor-lock", "")

def update():

    global live_plot
    global canvas
    global hexdump
    global last_time
    global last_bytes
    global alpha
    global last_bps
    global channelFilter

    display_bytes = 16*32
    display_packets = 100
    plot_bytes = 100

    while arduino.is_open and arduino.inWaiting():
        byte = int.from_bytes(arduino.read(), byteorder='big')
        parsing_deque.append(byte)
        input_buffer.append(byte)

    if len(parsing_deque) > 0:
        for p in rvtr.parse(parsing_deque):
            packets.append(p)

    try:
        hexdump.delete(1.0, tk.END)
        start = len(input_buffer) - display_bytes;
        if start < 0:
            start = 0
        start = int(np.ceil(start/16.0)*16);
        hexdumpString = ""
        i = 0
        row = int(start/16)
        toPrint = input_buffer[start:]
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

        hexdump.insert(tk.END, hexdumpString)
        hexdump.see(tk.END)
    except:
        pass

    try:
        packetOutput.delete(1.0, tk.END)
        output = ""
        index = 0
        for p in packets:
            id = p[3]
            if (channelFilter.get() == "All Channels") or (channelDict[channelFilter.get()] == id):
                output += "{:04X}".format(index) + "   "
                output += channels[id].ljust(max_channel_length + 2)
                output += " ".join("{:02X}".format(x) for x in p) + "\n"
            index += 1
        packetOutput.insert(tk.END, output)
        packetOutput.see(tk.END)
    except:
        pass

    try:
        live_plot.clear()
        toPlot = input_buffer[-plot_bytes:]
        elements = len(toPlot)
        t = np.arange(len(input_buffer) - elements, len(input_buffer), 1)
        live_plot.plot(t, toPlot)
        live_plot.set_title("Raw Incoming Bytes")
        canvas.draw()
    except:
        pass

    currTime = datetime.utcnow();
    currentTime["text"] = str(currTime) + " UTC"
    freq = np.round(100/(time.time() - last_time))/100.0
    frequency["text"] = "{0:.2f}".format(freq) + " Hz"

    bps = ((len(input_buffer) - last_bytes)/(time.time() - last_time))
    bps = alpha*bps + (1 - alpha)*last_bps

    bytesPerSecond["text"] = "{0:.1f}".format(np.round(bps*10)/10.0) + " B/s"
    last_time = time.time()
    last_bytes = len(input_buffer)
    last_bps = bps;

    main_window.after(40, update)

def get_com_ports():
    global portselect
    ports = []
    devices = list_ports.comports(True)
    for x in devices:
        ports.append(x.device)
    try:
        portselect['values'] = ports
        portselect.grid()
    except:
        pass
    return ports

def clear_buffers():
    global input_buffer
    global packets
    input_buffer = list([])
    packets = list([])

channelDict = {
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

channels = list([
    "/null",
    "/ground/ping",
    "/rocket/ping",
    "/relay/ping",
    "/rocket/console",
    "/relay/rocket/console",
    "/relay/console",
    "/rocket/status",
    "/relay/rocket/status",
    "/ground/fill-nitrous",
    "/ground/disconnect-feed-line",
    "/rocket/imudata",
    "/relay/rocket/imudata",
    "/ground/all-stop",
    "/ground/reset",
    "/ground/perform-test",
    "/ground/relay-mode",
    "/relay/ground/relay-mode",
    "/rocket/voltage",
    "/rocket/motor-info",
    "/rocket/motor-unlock",
    "/rocket/motor-lock"
])

max_channel_length = max([len(x) for x in channels])

baudrates = list([
    19200, 38400, 57600, 115200, 230400, 460800, 500000, 576000, 921600
])

arduino = serial.Serial()
print("Starting R@VT control.")

last_time = time.time();
parsing_deque = deque([])
input_buffer = list([])
packets = list([])
last_bytes = 0
last_bps = 0
alpha = 0.02

main_window = tk.Tk()
main_window.title("Ground Control")
window_icon = tk.PhotoImage(file='glider.ico')
main_window.tk.call('wm', 'iconphoto', main_window._w, window_icon)
main_window.resizable(width=False, height=False)

main_frame = tk.Frame(main_window, borderwidth=2, relief='groove')

# other windows -------------------------------------------------------

windowsFrame = tk.Frame(main_frame)

tk.Button(windowsFrame, text='Hexdump', width=12,
command=lambda: open_hexdump()).grid(row=1, column=0, padx=5, pady=5)

tk.Button(windowsFrame, text='Packet Viewer', width=12,
command=lambda: open_packet_window()).grid(row=1, column=1, padx=5, pady=5)

tk.Button(windowsFrame, text='Plotter', width=12,
command=lambda: open_plotter()).grid(row=1, column=2, padx=2, pady=5)

tk.Button(windowsFrame, text='Clear Buffers', width=12,
command=lambda: clear_buffers()).grid(row=2, column=1, padx=5, pady=5)

windowsFrame.grid(padx=5, pady=5)

# sending stuff ------------------------------------------------------

commsFrame = tk.Frame(main_frame)

tk.Label(commsFrame, text="Send commands to the rocket " +
"via this dialogue.").grid(row=0, columnspan=3)

channelSend = ttk.Combobox(commsFrame, value=channels, width=30)
channelSend.current(0)
channelSend.grid(row=2, column=0, columnspan=2, padx=5, pady=5)

textInputBox = tk.Entry(commsFrame, width=50)
textInputBox.grid(row=1, columnspan=3)
tk.Button(commsFrame, text='Send', width=10, command=lambda:
send_command(channelSend.get(), textInputBox.get())).grid(row=2, column=2, padx=5, pady=5)

ports=get_com_ports()
portselect = ttk.Combobox(commsFrame, values=ports)
if len(ports) > 0:
    portselect.current(0)
portselect.grid(row=3, column=0, padx=5, pady=5)

tk.Button(commsFrame, text='Update Ports', width=12,
command=lambda: get_com_ports()).grid(row=3, column=1, padx=5, pady=5)

baudselect = ttk.Combobox(commsFrame, values=baudrates)
baudselect.current(3)
baudselect.grid(row=4, columnspan=3, padx=5, pady=5)

openButton = tk.Button(commsFrame, text='Open', width=10, command=lambda:
open_port(portselect.get(), baudselect.get()))
openButton.grid(row=3, column=2, padx=5, pady=5)

commsFrame.grid(padx=5, pady=5)

# unlock frame ------------------------------------------------------

unlockFrame = tk.Frame(main_frame)

unlock3 = tk.IntVar()
unlock2 = tk.IntVar()
unlock1 = tk.IntVar()

checkbox3 = tk.Checkbutton(unlockFrame, text='Unlock 3',
state=tk.DISABLED, variable=unlock3,
command=lambda:
handle_motor_lock(unlock1, unlock2, unlock3,
checkbox1, checkbox2, checkbox3, 3))
checkbox3.grid(column=2, row=0, padx=5, pady=5)

checkbox2 = tk.Checkbutton(unlockFrame, text='Unlock 2',
state=tk.DISABLED, variable=unlock2,
command=lambda:
handle_motor_lock(unlock1, unlock2, unlock3,
checkbox1, checkbox2, checkbox3, 2))
checkbox2.grid(column=1, row=0, padx=5, pady=5)

checkbox1 = tk.Checkbutton(unlockFrame, text='Unlock 1', variable=unlock1,
command=lambda:
handle_motor_lock(unlock1, unlock2, unlock3,
checkbox1, checkbox2, checkbox3, 1))
checkbox1.grid(column=0, row=0, padx=5, pady=5)

unlockFrame.grid(padx=5, pady=5)

dataFrame = tk.Frame(main_frame)

currentTime = tk.Label(dataFrame, text="Time", font=("Consolas", 10))
currentTime.grid(row=1, columnspan=2, padx=5, pady=5)

frequency = tk.Label(dataFrame, text="Frequency", font=("Consolas", 10))
frequency.grid(row=1, column=2, padx=5, pady=5)

bytesPerSecond = tk.Label(dataFrame, width=10, text="BPS", anchor=tk.E, justify=tk.RIGHT, font=("Consolas", 10))
bytesPerSecond.grid(row=1, column=3, padx=5, pady=5)

dataFrame.grid(padx=5, pady=5)

update()
main_frame.grid(padx=5, pady=5)
main_window.grid()
main_window.mainloop()
