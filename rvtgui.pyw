from tkinter import *
from tkinter.ttk import *
from tkinter.font import Font
from tkinter.scrolledtext import ScrolledText
import time
from datetime import datetime
import numpy
import os
import socket
import re
import copy

CONSOLE_FONT = ("Consolas", 14)

default_nodes = [
    "/exec",
    "/hardware/driver_dispatcher",
    "/hardware/ematch",
    "/hardware/ignition_valve",
    "/hardware/linear_actuator",
    "/hardware/solenoid",
    "/hardware/vent_valve",
    "/listener",
    "/logging/rosbag_record",
    "/readiness_admin",
    "/rosnode_lister",
    "/rosout",
    "/sensors/combustion_thermocouple_1",
    "/sensors/combustion_thermocouple_2",
    "/sensors/combustion_transducer",
    "/sensors/float_switch",
    "/sensors/ox_tank_thermocouple",
    "/sensors/ox_tank_transducer",
    "/sensors/sensor_monitor",
    "/tcp_server",
    "/watchdog"
]
set_of_nodes = set()
active_nodes = set()
node_filters = []

config_window = None
def make_config_window():
    global config_window
    if not config_window:
        config_window = ConfigWindow()
    else:
        make_focus(config_window)

button_commands = [

    "system fortune | cowsay",
    "rosnode list",
    "rostopic list",
    ("View Storage Usage", "system df -h"),
    ("View Memory Usage", "system free -th"),
    ("View Logs", "system ls -lh ~/rocket-os/src/launch/logs"),
    ("View CPU Usage", "system mpstat -P ALL 1 1"),
    "",
    "read data",
    "print whitelist",
    "set readiness 0",
    "set readiness 10",
    "",
    "all stop",
    "close solenoid",
    "open solenoid",
    "close ignition valve",
    "open ignition valve",
    "retract linear actuator",
    "extend linear actuator",
    "close vent valve",
    "open vent valve",
    "crack vent valve",
    "fire ematch",
    "",
    "launch",
    "abort",
    ("Clean Shutdown", "fork rosnode kill -a")
]

history_index = 0
command_history = []

def make_command_button(master, window, command, row):

    if not command:
        Separator(window, orient=HORIZONTAL
        ).pack(side=TOP, pady=5, fill='x')
        return

    command = copy.copy(command)
    label = command
    if type(command) is tuple:
        label = command[0]
        command = command[1]
    row = copy.copy(row)
    Button(window,
        text=label,
        command=lambda: master.send_command(command),
        width=25
    ).pack(side=TOP, padx=5, pady=2)

def dict2str(dict, debug, info, warn, error, fatal,
    show_level=False, show_time=False, show_node=False):

    if dict["node"] not in active_nodes:
        return ""

    level = dict["level"]
    if level == "DEBUG" and not debug:
        return ""
    if level == "INFO" and not info:
        return ""
    if level == "WARN" and not warn:
        return ""
    if level == "ERROR" and not error:
        return ""
    if level == "FATAL" and not fatal:
        return ""

    ret = dict['content'] + "\n"
    if show_level or show_time or show_node:
        ret = ": " + ret
    header = []
    if show_level:
        header.append(("[" + level + "]").ljust(7))
    if show_time:
        header.append("[" + dict['stamp'] + "]")
    if show_node:
        header.append("[" + dict['node'] + "]")

    return ' '.join(header) + ret


def make_focus(window):
    window.attributes('-topmost', 1)
    window.attributes('-topmost', 0)
    window.focus()

class MainWindow(Tk):

    def __init__(self):

        self.delay = 50
        self.buffer = list([])
        self.socket = None
        self.num_shown = 0
        self.addr = None
        self.port = None
        self.logfile = None

        if not os.path.exists("logs"):
            os.mkdir("logs")


        Tk.__init__(self)
        self.style = Style()
        self.style.theme_use("xpnative")
        self.style.configure("console", foreground="black", background="white")
        self.title("Rocketry@VT Launch Control Operator Interface v2020-02-22a")
        self.wm_iconbitmap("logo_nowords_cZC_icon.ico")
        self.protocol("WM_DELETE_WINDOW", self.destroy)
        make_focus(self)
        self.update_idletasks()
        width = 1400
        height = 800
        self.geometry('{}x{}'.format(width, height))
        self.state('zoomed')

        bottom_frame = Frame(self)
        sidebar = Frame(self)
        top_frame = Frame(self)
        self.filter_frame = Frame(self)
        Label(self.filter_frame,
            text="Filter by Node").pack(side=TOP, anchor='w', padx=(4, 10), pady=2)
        self.status_text = Label(self)
        self.set_status("Disconnected.")

        for i in range(0, len(button_commands)):
            make_command_button(self, sidebar, button_commands[i], i)
        sidebar.pack(side=LEFT, fill=BOTH)
        self.status_text.pack(side=TOP, fill='x', padx=3, pady=6)

        Label(top_frame, text="IP Address: ").pack(side = LEFT, padx=3, pady=3)
        self.addrInputBox = Entry(top_frame, width=30)
        self.addrInputBox.insert(0, "spookyscary.ddns.net")
        self.addrInputBox.pack(side = LEFT, padx=3, pady=3)
        Label(top_frame, text="Port: ").pack(side = LEFT, padx=3, pady=3)
        self.portInputBox = Entry(top_frame, width=10)
        self.portInputBox.insert(0, "8001")
        self.portInputBox.pack(side = LEFT, padx=3, pady=3)
        self.connectButton = Button(top_frame, text='Connect',
            command=lambda: self.tcp_connect(self.addrInputBox.get(), self.portInputBox.get()))
        self.connectButton.pack(side = LEFT, padx=3, pady=3)
        clearButton = Button(top_frame, text='Clear',
            command=lambda: self.clear_console())
        # configButton = Button(top_frame, text='Filter Messages',
        #     command=lambda: make_config_window())
        # configButton.pack(side = LEFT, padx=3, pady=3)
        clearButton.pack(side = LEFT, padx=3, pady=3)

        self.snap_to_bottom = BooleanVar()
        self.snap_to_bottom.set(True)
        self.show_debug = BooleanVar()
        self.show_debug.set(False)
        self.show_debug.trace('w', self.redraw_console)
        self.show_info = BooleanVar()
        self.show_info.set(True)
        self.show_info.trace('w', self.redraw_console)
        self.show_warn = BooleanVar()
        self.show_warn.set(True)
        self.show_warn.trace('w', self.redraw_console)
        self.show_error = BooleanVar()
        self.show_error.set(True)
        self.show_error.trace('w', self.redraw_console)
        self.show_fatal = BooleanVar()
        self.show_fatal.set(True)
        self.show_fatal.trace('w', self.redraw_console)
        self.show_level = BooleanVar()
        self.show_level.set(True)
        self.show_level.trace('w', self.redraw_console)
        self.show_time = BooleanVar()
        self.show_time.set(True)
        self.show_time.trace('w', self.redraw_console)
        self.show_node = BooleanVar()
        self.show_node.set(True)
        self.show_node.trace('w', self.redraw_console)

        Checkbutton(top_frame, text="Snap to Bottom", var=self.snap_to_bottom,
            ).pack(side = LEFT, padx=3, pady=3)
        Checkbutton(top_frame, text="Debug", var=self.show_debug,
            ).pack(side = LEFT, padx=3, pady=3)
        Checkbutton(top_frame, text="Info", var=self.show_info,
            ).pack(side = LEFT, padx=3, pady=3)
        Checkbutton(top_frame, text="Warn", var=self.show_warn,
            ).pack(side = LEFT, padx=3, pady=3)
        Checkbutton(top_frame, text="Error", var=self.show_error,
            ).pack(side = LEFT, padx=3, pady=3)
        Checkbutton(top_frame, text="Fatal", var=self.show_fatal,
            ).pack(side = LEFT, padx=3, pady=3)
        Checkbutton(top_frame, text="Levels", var=self.show_level,
            ).pack(side = LEFT, padx=3, pady=3)
        Checkbutton(top_frame, text="Timestamp", var=self.show_time,
            ).pack(side = LEFT, padx=3, pady=3)
        Checkbutton(top_frame, text="Nodes", var=self.show_node,
            ).pack(side = LEFT, padx=3, pady=3)
        top_frame.pack(side=TOP, fill='x')
        self.filter_frame.pack(side=RIGHT, fill='y')

        self.textOutput = ScrolledText(self, wrap=CHAR,
            width = 28*3, bg='#1A3747', fg='white',
            relief='flat', borderwidth=6, font=CONSOLE_FONT)
        self.textOutput.pack(fill=BOTH, expand=YES)
        self.textOutput.config(state=DISABLED)

        self.textOutput.tag_configure("DEBUG", foreground="#5D737E")
        self.textOutput.tag_configure("INFO", foreground="white")
        self.textOutput.tag_configure("WARN", foreground="#fffe00")
        self.textOutput.tag_configure("ERROR", foreground="#f08e00")
        self.textOutput.tag_configure("FATAL", foreground="#bb0000")

        self.textOutput.bind("<MouseWheel>", self.on_text_scroll)

        self.textInputBox = Entry(bottom_frame, width=70)
        self.textInputBox.pack(padx=5, pady=5, fill=BOTH)
        bottom_frame.pack(side=BOTTOM, fill=BOTH)

        self.bind("<Return>", self.pressed_enter)
        self.textInputBox.bind("<Up>", self.pressed_arrow_key)
        self.textInputBox.bind("<Down>", self.pressed_arrow_key)

        for node in default_nodes:
            self.register_node(node)

        self.grid_columnconfigure(0, weight=1)
        self.grid_rowconfigure(0, weight=1)
        self.begin_loop()

    def on_text_scroll(self, event):
        if event.state == 4:
            font = self.textOutput['font']
            typeface, size = font.split()
            if event.delta < 0:
                size = int(size) - 1
            else:
                size = int(size) + 1
            self.textOutput['font'] = typeface + " " + str(size)

            if self.snap_to_bottom.get():
                self.textOutput.see(END)


    def set_status(self, message):
        self.status_text['text'] = message

    def begin_loop(self):

        self.update()
        self.after(self.delay, self.begin_loop)

    def update(self):

        message = b""

        if self.socket is not None:
            self.set_status("Connected at {}:{}. Recieved {} messages (showing {})."
                .format(self.addr, self.port,
                    len(self.buffer), self.num_shown))
            while True:
                part = b""
                try:
                    part = self.socket.recv(1000)
                except:
                    pass
                message += part
                if len(part) < 1000:
                    break
        else:
            self.set_status("Disconnected.")

        if len(message) > 0:

            if not self.logfile:
                filename = "logs/LOG-" + str(datetime.utcnow()
                    ).replace(" ", "-").replace(":", "-") + ".txt"
                self.logfile = open(filename, 'wb')

            message = message.decode('utf-8', 'ignore')
            self.logfile.write(message.encode())
            self.logfile.flush()
            parsed = self.parse_message(message)
            for p in parsed:
                self.buffer.append(p)

                st = dict2str(p,
                    self.show_debug.get(),
                    self.show_info.get(),
                    self.show_warn.get(),
                    self.show_error.get(),
                    self.show_fatal.get(),
                    self.show_level.get(),
                    self.show_time.get(),
                    self.show_node.get())

                if st:
                    self.num_shown += 1

                    begin = "end-10c linestart"
                    end = "end-10c lineend"
                    self.textOutput.configure(state='normal')
                    self.textOutput.insert(END, st)
                    self.textOutput.configure(state='disabled')
                    if p['level'] == "DEBUG":
                        self.textOutput.tag_add("DEBUG", begin, end)
                    if p['level'] == "INFO":
                        self.textOutput.tag_add("INFO", begin, end)
                    if p['level'] == "WARN":
                        self.textOutput.tag_add("WARN", begin, end)
                    if p['level'] == "ERROR":
                        self.textOutput.tag_add("ERROR", begin, end)
                    if p['level'] == "FATAL":
                        self.textOutput.tag_add("FATAL", begin, end)

                    if self.snap_to_bottom.get():
                        self.textOutput.see(END)


    def tcp_connect(self, addr, port):

        self.addr = addr
        self.port = port
        self.set_status("Connecting...")
        self.socket = socket.socket()
        if self.connectButton["text"] == "Disconnect":
            self.connectButton.config(text="Connect")
            self.set_status("Disconnected.")
            self.socket = None
            return
        try:
            self.socket.connect((addr, int(port)))
        except Exception as e:
            self.set_status(str(e))
            return
        self.socket.setblocking(0)
        self.connectButton.config(text="Disconnect")
        self.set_status("Connected at {}:{}.".format(addr, port))

    def send_command(self, text):
        global history_index
        try:
            if not len(command_history) or command_history[-1] != text:
                command_history.append(text)
            history_index = len(command_history)
            self.socket.sendall(text.encode())
        except Exception as e:
            print(e)

    def pressed_enter(self, event):
        if self.focus_get() is self.textInputBox:
            self.send_command(self.textInputBox.get())
            self.textInputBox.delete(0, 'end')

    def pressed_arrow_key(self, event):
        global history_index
        if event.keysym == "Up":
            history_index -= 1
            if history_index < 0:
                history_index = 0
            cmd = command_history[history_index]
            self.textInputBox.delete(0, 'end')
            self.textInputBox.insert(0, cmd)
        elif event.keysym == "Down":
            history_index += 1
            cmd = ""
            if history_index > len(command_history):
                history_index = len(command_history)
            if history_index < len(command_history):
                cmd = command_history[history_index]
            self.textInputBox.delete(0, 'end')
            self.textInputBox.insert(0, cmd)

    def clear_buffers(self):
        self.buffer.clear()

    def clear_console(self):
        self.textOutput.configure(state='normal')
        self.textOutput.delete('1.0', 'end')
        self.textOutput.configure(state='disabled')
        self.buffer = []
        self.num_shown = 0

    def redraw_console(self, event=None, another=None, more=None):
        self.textOutput.configure(state='normal')
        self.textOutput.delete('1.0', 'end')
        self.textOutput.configure(state='disabled')
        self.num_shown = 0
        for p in self.buffer:
            st = dict2str(p,
                self.show_debug.get(),
                self.show_info.get(),
                self.show_warn.get(),
                self.show_error.get(),
                self.show_fatal.get(),
                self.show_level.get(),
                self.show_time.get(),
                self.show_node.get())
            if st:
                self.num_shown += 1
                begin = "end-10c linestart"
                end = "end-10c lineend"
                self.textOutput.configure(state='normal')
                self.textOutput.insert(END, st)
                self.textOutput.configure(state='disabled')
                if p['level'] == "DEBUG":
                    self.textOutput.tag_add("DEBUG", begin, end)
                if p['level'] == "INFO":
                    self.textOutput.tag_add("INFO", begin, end)
                if p['level'] == "WARN":
                    self.textOutput.tag_add("WARN", begin, end)
                if p['level'] == "ERROR":
                    self.textOutput.tag_add("ERROR", begin, end)
                if p['level'] == "FATAL":
                    self.textOutput.tag_add("FATAL", begin, end)

                if self.snap_to_bottom.get():
                    self.textOutput.see(END)


    def toggle_node(self, nodename):

        if nodename in active_nodes:
            active_nodes.remove(nodename)
        else:
            active_nodes.add(nodename)
        self.redraw_console()

    def add_node_checkbox(self, nodename):

        nodename = copy.copy(nodename)
        var = IntVar()
        var.set(1)
        node_filters.append(var)
        cb = Checkbutton(self.filter_frame,
            text=nodename,
            var=var,
            command=lambda: self.toggle_node(nodename))
        cb.pack(side=TOP, anchor='w', padx=(4, 10), pady=2)

    def parse_message(self, string):

        pattern = re.compile(
            "\[\w+\]\s+\[\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2}\.\d{6}\] \[.+\]\:[\S\s]+?(?=(?:\[DEBUG\]|\[INFO\]|\[WARN\]|\[ERROR\]|\[FATAL\]|\n\[DEBUG\]|\n\[INFO\]|\n\[WARN\]|\n\[ERROR\]|\n\[FATAL\]|$))")
        text = re.compile("\]:(\s|$)([\S\s]+)")
        meta = re.compile("\[(.*?)\]")

        messages = pattern.findall(string);

        msgdicts = []
        for msg in messages:
            content = text.findall(msg)[0][1]

            metadata = meta.findall(msg)
            (level, datestr, node) = metadata[0:3]
            dict = {}
            dict["content"] = content
            dict["level"] = level
            dict["stamp"] = datestr
            dict["node"] = node
            self.register_node(node)

            msgdicts.append(dict)

        return msgdicts

    def register_node(self, node):

        if node not in set_of_nodes:
            set_of_nodes.add(node)
            active_nodes.add(node)
            self.add_node_checkbox(node)


class ConfigWindow(Tk):

    def __init__(self):

        self.delay = 50

        Tk.__init__(self)
        self.style = Style()
        self.style.theme_use("xpnative")
        self.style.configure("console", foreground="black", background="white")
        self.title("Filter Messages")
        self.wm_iconbitmap("logo_nowords_cZC_icon.ico")
        self.protocol("WM_DELETE_WINDOW", self.close)
        make_focus(self)
        self.update_idletasks()
        # width = 300
        # height = 400
        # self.geometry('{}x{}'.format(width, height))

        frame = Frame(self, relief='groove')

        import string
        import random
        letters = string.ascii_lowercase
        nodes = []
        for i in range(15):
            nodes.append(''.join(random.choice(letters) for i in range(14)))
        levels = ["DEBUG", "INFO", "WARN", "ERROR", "FATAL"]

        for i, node in enumerate(nodes):
            Checkbutton(frame, text=node
            ).grid(row=i, column=0, padx=4, pady=4, sticky='w')
            for j, level in enumerate(levels):
                Checkbutton(frame, text=level
                ).grid(row=i, column=j+1, padx=4, pady=4, sticky='w')


        frame.pack(fill=BOTH, padx=2, pady=2)

        self.begin_loop()

    def begin_loop(self):
        self.update()
        self.after(self.delay, self.begin_loop)

    def update(self):
        pass

    def close(self):
        global config_window
        config_window = None
        self.destroy()


print("Starting R@VT control.")

main = MainWindow()
main.mainloop()
