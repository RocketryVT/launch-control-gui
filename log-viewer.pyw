
import tkinter
from tkinter import filedialog
import os
import ctypes
from datetime import datetime
from collections import deque
import string
import rvtr
import rvtgui

myappid = 'log-viewer' # arbitrary string
ctypes.windll.shell32.SetCurrentProcessExplicitAppUserModelID(myappid)

class MainWindow(tkinter.Tk):

    def __init__(self):

        self.filename = '';

        tkinter.Tk.__init__(self)
        self.title("Log Viewer")
        self.wm_iconbitmap("launch.ico")
        self.tk.call('wm', 'iconphoto', self._w,
        tkinter.PhotoImage(file='launch.ico'))
        self.protocol("WM_DELETE_WINDOW", self.destroy)
        self.attributes('-topmost', 1)
        self.attributes('-topmost', 0)
        self.focus()

        menu = tkinter.Menu(self)
        file = tkinter.Menu(menu, tearoff=False)
        file.add_command(label="Open", command=self.open_file)
        menu.add_cascade(label="File", menu=file)
        self.config(menu=menu)

        self.output = tkinter.Text(self, width=90,
        height=30, wrap="none", font=("Droid Sans Mono", 12))
        self.output.tag_config("error", background="#FFDDDD")

        scroll = tkinter.Scrollbar(self, command=self.output.yview)
        self.output['yscrollcommand'] = scroll.set
        scroll.pack(side=tkinter.RIGHT, fill=tkinter.Y)
        self.output.pack(expand=True, fill=tkinter.BOTH);

        self.mainloop()

    def open_file(self):

        self.filename =  tkinter.filedialog.askopenfilename(
        initialdir = ".", title = "Select file")
        self.redraw()

    def redraw(self):

        if self.filename == '':
            self.title("Log Viewer")
            return

        self.title("Log Viewer - " + self.filename)
        parse_buffer = deque(open(self.filename, 'rb').read())
        packets, error = rvtr.parse(parse_buffer)
        output = ""
        index = 0
        ascii = True
        error_lines = list([])
        for p in packets:
            id = p[3]
            if p[-1] == ord('!') and p[-2] == ord('!'):
                error_lines.append(index+1)
            try:
                channel = list(rvtr.CHANNELS.keys())[
                list(rvtr.CHANNELS.values()).index(id)]
            except:
                channel = "(" + str(id) + ")"
            output += "{:04X}".format(index) + "   "
            output += ("[" + channel + "] ").ljust(35)
            if ascii:
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
        self.output.delete(1.0, tkinter.END)
        self.output.insert(tkinter.END, output)
        for line in error_lines:
            self.output.tag_add("error",
            "{}.0".format(line), "{}.end".format(line))

MainWindow()
