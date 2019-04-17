# -*- coding: utf-8 -*-
"""
Created on Fri Apr 12 15:10:36 2019

@author: Alejandro
"""

#This cals for matplotlib 
import matplotlib
#this is a backend for matplotlib 
matplotlib.use("TKAgg")
#this allows us to draw a graph on matplot lib aswell as have a navigation tool bar for the graph 
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
from matplotlib.figure import Figure
#this is importing the animation function from matplotlib
import matplotlib.animation as animation 
#this imports different styles for the graph...some can be personalized
from matplotlib import style
#this calls for all the functionalities
import tkinter as tk 
#This is for styling the GUI...like the buttons 
from tkinter import ttk
import math
import transmission
import time

text_file = open("C:\\Users\\Alejandro\\Documents\\School\\VTech Rocketry\\Final\\Bytes.txt")
line = 0

#------------------------------------------------------------------------------

#------------------------------------------------------------------------------
RocketryAPP = tk.Tk()
RocketryAPP.title('Controls')
RocketryAPP.geometry("1280x720")
tk.Tk.iconbitmap(RocketryAPP, "favicon.ico")
#------------------------------------------------------------------------------
topFrame = tk.Frame(RocketryAPP)
topFrame.pack(side='top', fill='x')


def create_window():
    BYTES = tk.Toplevel(RocketryAPP)
    BYTES.title('BYTES')
    BYTES.geometry("1280x720")
    
    
    output = tk.Label(BYTES, text = "Output\n", font=("Consolas", 16), justify="left")   
    
    starttime=time.time()
    while True:
        time.sleep(3)
        print("Happy")
        
    text_Input = tk.StringVar()
    text_Output = tk.StringVar()
    txtEntry = tk.Entry(BYTES, textvariable=text_Input)
    txtEntry.pack(pady=4, padx=4)
    output.pack()
      
    EnterButton = tk.Button(BYTES, text='Enter', command=copy)
    EnterButton.pack(pady=4, padx=4)

#-----------------------------------------------------------------------------
Button_1 = ttk.Button(topFrame, text = "Button 1")
Button_1.pack(side='left', pady=4, padx=4)

Button_2 = ttk.Button(topFrame, text = "Button 2")
Button_2.pack(side='left', pady=4, padx=4)

Button_3 = ttk.Button(topFrame, text = "Button 3")
Button_3.pack(side='left', pady=4, padx=4)

Button_4 = ttk.Button(topFrame, text = "Button 4")
Button_4.pack(side='left', pady=4, padx=4)

Button_5 = ttk.Button(topFrame, text = "Button 5")
Button_5.pack(side='left', pady=4, padx=4)

Button_6 = ttk.Button(topFrame, text = "Button 6")
Button_6.pack(side='left', pady=4, padx=4)

Button_7 = ttk.Button(topFrame, text = "BYTES", command=create_window)
Button_7.pack(side='left', pady=4, padx=4)

#Checkbox----------------------------------------------------------------------

Var1 = tk.IntVar()
tk.Checkbutton(topFrame, text='Unlock 1', variable = Var1).pack(side='left')

Var2 = tk.IntVar()
tk.Checkbutton(topFrame, text='Unlock 2', variable = Var2).pack(side='left')

Var3 = tk.IntVar()
tk.Checkbutton(topFrame, text='Unlock 3', variable = Var3).pack(side='left')
#ENTRY------------------------------------------------------------------------
sideFrame = tk.Frame(RocketryAPP)
sideFrame.pack(side='right', fill='y')

output = tk.Label(text = "Output\n", font=("Consolas", 16), justify="left")

def copy(arg=None):
    temp = txtEntry.get()
    # txtOutput.insert(0, temp)
    chunk_size = 50
    chunks = math.ceil(len(temp)/chunk_size)
    output["text"] += ">"
    for i in range(0, chunks):
        output["text"] += temp[i:i+chunk_size] + "\n"
    

text_Input = tk.StringVar()
text_Output = tk.StringVar()
txtEntry = tk.Entry(sideFrame, textvariable=text_Input)
txtEntry.pack(pady=4, padx=4)

EnterButton = tk.Button(sideFrame, text='Enter', command=copy)
EnterButton.pack(pady=4, padx=4)
output.pack()
#Window------------------------------------------------------------------------


#Graph-------------------------------------------------------------------------


f = Figure()
a = f.add_subplot(111)
a.plot([1,2,3,4,5,6], [2,5,4,3,8,9])

canvas = FigureCanvasTkAgg(f, RocketryAPP)
canvas.draw()
canvas.get_tk_widget().pack()

toolbar = NavigationToolbar2Tk(canvas, RocketryAPP)
toolbar.update()
canvas._tkcanvas.pack(side=tk.TOP, fill=tk.BOTH, expand = True)

RocketryAPP.mainloop()




RocketryAPP.mainloop()