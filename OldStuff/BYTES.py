import matplotlib
matplotlib.use("TKAgg")
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
from matplotlib.figure import Figure
import tkinter as tk
from tkinter import ttk
import math
import time

text_file = open("./data/Bytes.txt")
line = 0


RocketryAPP = tk.Tk()
RocketryAPP.title('Controls')
RocketryAPP.geometry("1280x720")

topFrame = tk.Frame(RocketryAPP)
topFrame.pack(side='top', fill='x')


def send_command(f, text):
    try:
        print("send")
        f.write(text)
    except ValueError:
        pass

def create_window():
    BYTES = tk.Toplevel(RocketryAPP)
    BYTES.title('BYTES')
    BYTES.geometry("600x400")
    output = tk.Label(BYTES, text = "Output\n", font=("Consolas", 16), justify="left" )

    commandFile = open("./writecommand.txt", 'w')
    text_Input = tk.StringVar()
    text_Output = tk.StringVar()
    txtEntry = tk.Entry(BYTES, textvariable=text_Input)
    txtEntry.pack(pady=4, padx=4)
    output.pack()
      
    EnterButton = tk.Button(BYTES, text='Enter', command=lambda: send_command(commandFile, text_Input.get()))
    EnterButton.pack(pady=4, padx=4)

Button_7 = ttk.Button(topFrame, text = "BYTES", command=create_window)
Button_7.pack(side='left', pady=4, padx=4)

# Checkbox----------------------------------------------------------------------

Var1 = tk.IntVar()
tk.Checkbutton(topFrame, text='Unlock 3', variable=Var1, state=tk.DISABLED).pack(side='right')

Var2 = tk.IntVar()
tk.Checkbutton(topFrame, text='Unlock 2', variable=Var2, state=tk.DISABLED).pack(side='right')

Var3 = tk.IntVar()
tk.Checkbutton(topFrame, text='Unlock 1', variable=Var3).pack(side='right')
# ENTRY------------------------------------------------------------------------

# Window------------------------------------------------------------------------


# Graph-------------------------------------------------------------------------

n = ttk.Notebook(RocketryAPP)
f1 = ttk.Frame(n)
f2 = ttk.Frame(n)
n.add(f1, text='One')
n.add(f2, text='Two')

f = Figure()
a = f.add_subplot(111)
a.plot([1,2,3,4,5,6], [2,5,4,3,8,9])

canvas = FigureCanvasTkAgg(f, f1)
canvas.draw()
canvas.get_tk_widget().pack()

toolbar = NavigationToolbar2Tk(canvas, RocketryAPP)
toolbar.update()
canvas._tkcanvas.pack(side=tk.TOP, fill=tk.BOTH, expand = True)

RocketryAPP.mainloop()




RocketryAPP.mainloop()
