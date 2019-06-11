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
myappid = 'rvtgui' # arbitrary string
ctypes.windll.shell32.SetCurrentProcessExplicitAppUserModelID(myappid)


def make_focus(window):
    window.attributes('-topmost', 1)
    window.attributes('-topmost', 0)
    window.focus()


class MainWindow(tkinter.Tk):

    def __init__(self, delay=50):

        self.delay = delay
        self.buffer = list([])
        self.buffer_size = 0
        self.arduino = serial.Serial()

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
        # self.resizable(width=False, height=False)
        make_focus(self)

        main_frame = tkinter.Frame(self, borderwidth=2, relief='groove')

        commsFrame = tkinter.Frame(main_frame)
        textInputBox = tkinter.Entry(commsFrame)
        textInputBox.grid(row=0, column=0, padx=5, pady=5)
        self.send_button = tkinter.Button(commsFrame, text='Send',
        state=tkinter.DISABLED, command=lambda:
        self.send_command(textInputBox.get()));
        self.send_button.grid(row=0, column=1, padx=5, pady=5)
        ports = self.refresh_ports()
        portselect = ttk.Combobox(commsFrame, values=ports)
        if len(ports) > 0:
            portselect.current(0)
        portselect.grid(row=0, column=2, padx=5, pady=5)
        tkinter.Button(commsFrame, text='Update Ports',
        command=lambda: self.refresh_ports(portselect)
        ).grid(row=0, column=3, padx=5, pady=5)
        baudselect = ttk.Combobox(commsFrame, values=baudrates)
        baudselect.current(3)
        baudselect.grid(row=0, column=4, padx=5, pady=5)
        openButton = tkinter.Button(commsFrame, text='Open', command=lambda:
        self.open_port(portselect.get(), baudselect.get(), openButton))
        openButton.grid(row=0, column=5, padx=5, pady=5)
        commsFrame.pack(side=tkinter.TOP, fill=tkinter.BOTH, expand=tkinter.YES)

        self.textOutput = tkinter.Text(self, wrap="none", width = 28*3,
        height=27, font=("Droid Sans Mono", 12), borderwidth=3, relief='sunken')
        self.textOutput.pack(side=tkinter.BOTTOM, fill=tkinter.BOTH, expand=tkinter.YES)

        main_frame.pack()
        self.grid_columnconfigure(0, weight=1)
        self.grid_rowconfigure(0, weight=1)
        self.begin_loop()

    def begin_loop(self):

        self.update()
        self.after(self.delay, self.begin_loop)

    def update(self):

        while self.arduino.is_open and self.arduino.inWaiting():
            byte = int.from_bytes(self.arduino.read(), byteorder='big')
            self.buffer.append(byte)
            if True:
                self.logfile.write(bytes([byte]))
                self.logfile.flush()

        if len(self.buffer) == self.buffer_size:
            return

        self.buffer_size = len(self.buffer)
        text = ""
        for i, x in enumerate(self.buffer):
            if chr(x) in string.printable:
                text += chr(x)
            else:
                text += "."

        self.textOutput.delete(1.0, tkinter.END)
        self.textOutput.insert(tkinter.END, text)
        self.textOutput.see(tkinter.END)

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

    def send_command(self, text):
        pass;
        packet = [0]
        for token in text.split(' '):
            try:
                packet.append(int(token, 16));
            except Exception as e:
                print("Failed to send:", e)
                return
        packet.append(0xFF)
        print(packet)

        self.buffer.append(ord('\n'));
        self.buffer.append(ord('>'));
        self.buffer.append(ord(' '));
        for num in packet:
            print(hex(num))
            s = hex(num)
            for ch in s:
                self.buffer.append(ord(ch))
            self.buffer.append(ord(' '));
        self.buffer.append(ord('\n'));

        if self.arduino.is_open:
            self.arduino.write(bytearray(packet))
            self.logfile.write(bytearray(packet))
        else:
            print("Cannot send; port not open")

    def clear_buffers(self):

        self.buffer.clear()


class DisplayWindow(tkinter.Toplevel):

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
        make_focus(self)
        self.begin_loop()

if __name__ == "__main__":

    print("Starting R@VT control.")

    MainWindow().mainloop()
