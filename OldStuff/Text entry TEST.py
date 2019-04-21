# -*- coding: utf-8 -*-
"""
Created on Fri Apr  5 14:48:40 2019

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
#------------------------------------------------------------------------------

#------------------------------------------------------------------------------
RocketryAPP = tk.Tk()
RocketryAPP.title('Controls')
RocketryAPP.geometry("1280x720")

#ENTRY------------------------------------------------------------------------

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
txtEntry = tk.Entry(textvariable=text_Input)
txtEntry.pack(pady=4, padx=4)

EnterButton = tk.Button(text='Enter', command=copy)
EnterButton.pack(pady=4, padx=4)
output.pack()

#----------------------------------------------------------------------------
sideFrame = tk.Frame(RocketryAPP)
sideFrame.pack(side='right', fill='y')

def copy(arg=None):
    temp = txtEntry.get()
    txtOutput.insert(0, temp)
    

text_Input = tk.StringVar()
text_Output = tk.StringVar()
txtEntry = tk.Entry(sideFrame, textvariable=text_Input)
txtEntry.pack(pady=4, padx=4)

EnterButton = ttk.Button(sideFrame, text='Enter', command=copy)
EnterButton.pack(pady=4, padx=4)

txtOutput = ttk.Entry(sideFrame, textvariable=text_Output)
txtOutput.pack(pady=4, padx=4)



# txtOutput = tk.Entry(textvariable=text_Output)
# txtOutput.pack(pady=4, padx=4)


RocketryAPP.mainloop()