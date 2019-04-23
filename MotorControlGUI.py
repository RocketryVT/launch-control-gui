import tkinter as tk
from tkinter import ttk
import matplotlib
matplotlib.use("TKAgg")
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
from matplotlib.figure import Figure


def send_command(f, text):
    try:
        print("send")
        s = text + "\n"
        f.write(s)
    except ValueError:
        pass


def activateCheck1(var, Chk2, Chk3,  chkvar2, chkvar3):
    if var:
        Chk2['state'] = tk.NORMAL
        print("Lock 1 Unlocked")
    else:
        Chk2['state'] = tk.DISABLED
        Chk3['state'] = tk.DISABLED
        chkvar2.set(0)
        chkvar3.set(0)
        print("Lock 1 Locked, Locks 2 and 3 relocked and disabled")

def activateCheck2(var, Chk3, chkvar3):
    if var:
        Chk3['state'] = tk.NORMAL
        print("Lock 2 Unlocked")
    else:
        Chk3['state'] = tk.DISABLED
        chkvar3.set(0)
        print("Lock 2 Locked, Lock 3 relocked and disabled")


commandFile = open("./writecommand.txt", 'w')
root = tk.Tk()
root.geometry("600x400")
topFrame = tk.Frame(root)
topFrame.pack(side='top', fill='x')

Var3 = tk.IntVar()
chk3 = tk.Checkbutton(topFrame, text='Unlock 3', variable=Var3, state=tk.DISABLED)
chk3.pack(side=tk.RIGHT, anchor='n')

Var2 = tk.IntVar()
chk2 = tk.Checkbutton(topFrame, text='Unlock 2', variable=Var2, state=tk.DISABLED, command=lambda: activateCheck2(Var2.get(), chk3, Var3))
chk2.pack(side=tk.RIGHT, anchor='n')

Var1 = tk.IntVar()
chk1 = tk.Checkbutton(topFrame, text='Unlock 1', variable=Var1, command=lambda: activateCheck1(Var1.get(), chk2, chk3,  Var2, Var3))
chk1.pack(side=tk.RIGHT, anchor='n')

note = ttk.Notebook(topFrame)
note.pack(side=tk.BOTTOM, expand=True, fill='both')

tab1 = tk.Frame(note)
tab2 = tk.Frame(note)
tab3 = tk.Frame(note)

# tab 1
text_Input = tk.StringVar()
text_Output = tk.StringVar()
txtEntry = tk.Entry(tab1, textvariable=text_Input, width=50)
txtEntry.pack()
EnterButton = tk.Button(tab1, text='Send', command=lambda: send_command(commandFile, text_Input.get()))
EnterButton.pack(pady=4, padx=4)

channels = ('All', 'Cannel 1', 'Channel 2')
cnames = tk.StringVar(value=channels)
lbox = tk.Listbox(tab1, listvariable=cnames)
lbox.pack(side=tk.LEFT, anchor='s')

# tab2
f = Figure()
a = f.add_subplot(111)
a.plot([1,2,3,4,5,6], [2,5,4,3,8,9])

canvas = FigureCanvasTkAgg(f, tab2)
canvas.draw()
canvas.get_tk_widget().pack()

toolbar = NavigationToolbar2Tk(canvas, tab2)
toolbar.update()
canvas._tkcanvas.pack(side=tk.TOP, fill=tk.BOTH, expand=True)

# tab3
tk.Button(tab3, text='Exit', command=root.destroy).pack(padx=100, pady=100)

note.add(tab1, text="Send/Receive Commands", compound=tk.TOP)
note.add(tab2, text="Graph 1")
note.add(tab3, text="Graph 2")
note.pack()
root.mainloop()
