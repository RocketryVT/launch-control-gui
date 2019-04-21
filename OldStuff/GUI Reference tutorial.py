# -*- coding: utf-8 -*-
"""
Created on Mon Jan 14 19:51:43 2019

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

from matplotlib import pyplot as plt

#this is specifying the font size and type 
ROMANFONT = ("Times New Roman", 12)
NORM_FONT = ("Times New Roman", 8)
SMALL_FONT = ("Verdana", 6)

#ggplot
style.use("ggplot")
#this is making a graph for Page three
f = Figure()
#subplot
a = f.add_subplot(111)

exchange = "Avionics"
DatCounter = 9000
programName = "avionics"

def avionicsTeam(toWhat, pn):
    global exchange
    global DatCounter
    global programName
    
    exchange = toWhat
    programName = pn
    DatCounter = 9000


#This is defining the popupmsg functionality
def popupmsg(msg):
    popup = tk.Tk()
    
    popup.wm_title("WARNING!")
    label = ttk.Label(popup, text=msg, font=NORM_FONT)
    label.pack(side="top", fill="x", pady=10)
    B1 = ttk.Button(popup, text="Okay", command=popup.destroy)
    B1.pack()
    popup.geometry("300x300")
    popup.mainloop()


#defining the function for animation for live preformance of the rocket
def animate(i):
    pullData = open("sampleData.txt","r").read()
    dataList = pullData.split('\n')
    xList = []
    yList = []
    for eachLine in dataList:
        if len(eachLine) > 1:
            x, y = eachLine.split(',')
            xList.append(int(x))
            yList.append(int(y))
    #important in order to avoid mess
    a.clear()
    a.plot(xList, yList)

#This is defining the "StartPage"
#this is defining the class for the GUI application 
#the '()' allows this class to inhereit attirbutes from another class
class Rocketryapp(tk.Tk): 
    
    #this is a method that is going to initalize this class...everytime the applciation runs, this will pop up first 
    def __init__(self, *args, **kwargs):
        #now we are going to add what we want to initilialize
        tk.Tk.__init__(self, *args, **kwargs)
        
#--------------I am trying to figure out how to change the window icon---------
        tk.Tk.iconbitmap(self, "favicon.ico")
#------------------------------------------------------------------------------
        
        #this is changing the title of the window
        tk.Tk.wm_title(self, "Rocketry GUI")
        
        #this is the main frame that will hold everything...the motherload 
        container = tk.Frame(self)
        #fills in the limit set and expand allows you to expand the limits 
        container.pack(side = "top", fill = "both", expand = True)
        #this is simple configuration...setting the bounds (minimum and who gets priority) 
        container.grid_rowconfigure(0, weight=1)
        container.grid_columnconfigure(0, weight=1)
        
        #this is for the main menu bar
        menubar = tk.Menu(container)
        filemenu = tk.Menu(menubar, tearoff=1)
        filemenu.add_command(label="Save settings", command = lambda: popupmsg("Not supported just yet!"))
        filemenu.add_separator()
        filemenu.add_command(label="Exit", command=quit)
        menubar.add_cascade(label="File", menu=filemenu)
        
        #This can include all the different subteams and the graphs they want to concentrate during testing
        #of during the flight of the rocket 
        subTeams = tk.Menu(menubar, tearoff=1)
        subTeams.add_command(label = "Avionics", 
                             command=lambda: avionicsTeam("Avionics", "avionics"))
        subTeams.add_command(label = "Propulsion", 
                             command=lambda: avionicsTeam("Propulsion", "propulsion"))
        subTeams.add_command(label = "Software", 
                             command=lambda: avionicsTeam("Software", "software"))
        subTeams.add_command(label = "Everyone", 
                             command=lambda: avionicsTeam("Everyone", "everyone"))
        menubar.add_cascade(label="SubTeams", menu=subTeams)
        
       

        tk.Tk.config(self, menu=menubar)        
        
        self.frames = {}
        
        for F in (StartPage, PageOne, PageTwo, PageThree):
        
            frame = F(container, self)
            
            self.frames[F] = frame
            #the sticky parameter means it keeps everything in place/aligned when expanding it 
            frame.grid(row = 0, column = 0, sticky="nsew" )
        
        self.show_frame(StartPage)
        
    def show_frame(self, cont):
        
        frame = self.frames[cont]
        
        frame.tkraise()
 

class StartPage(tk.Frame):
    
    def __init__(self, parent, controller):
        tk.Frame.__init__(self,parent)
        label = tk.Label(self, text="""This is a test to show that I can do this""", font=ROMANFONT)
        label.pack(padx=10)
    
        button1 = ttk.Button(self, text="Agree", 
                            command= lambda: controller.show_frame(PageThree))
        button1.pack()
        
        button2 = ttk.Button(self, text="Disagree", 
                            command=quit)
        button2.pack()
        


        
class PageOne(tk.Frame):
    
    def __init__(self, parent, controller):
        tk.Frame.__init__(self, parent)
        label = tk.Label(self, text="Page One", font=ROMANFONT)
        label.pack(padx=10)
        
        button1 = ttk.Button(self, text="Back to Start Page", 
                            command= lambda: controller.show_frame(StartPage))
        button1.pack()
        
        button2 = ttk.Button(self, text="Page Two", 
                            command= lambda: controller.show_frame(PageTwo))
        button2.pack()
        
        button3 = ttk.Button(self, text="Page Three", 
                            command= lambda: controller.show_frame(PageOne))
        button3.pack()

class PageTwo(tk.Frame):
    
    def __init__(self, parent, controller):
        tk.Frame.__init__(self, parent)
        label = tk.Label(self, text="Page Two", font=ROMANFONT)
        label.pack(padx=10)
        
        button1 = ttk.Button(self, text="Back to Start Page", 
                            command= lambda: controller.show_frame(StartPage))
        button1.pack()

        button2 = ttk.Button(self, text="Page One", 
                            command= lambda: controller.show_frame(PageOne))
        button2.pack()
        
        button3 = ttk.Button(self, text="Page Three", 
                            command= lambda: controller.show_frame(PageOne))
        button3.pack()

#this page is to have a graph as an example 
class PageThree(tk.Frame):
    
    def __init__(self, parent, controller):
        tk.Frame.__init__(self, parent)
        label = tk.Label(self, text="Page Three/Graph page", font=ROMANFONT)
        label.pack(padx=10)
        
        button1 = ttk.Button(self, text="Back to Start Page", 
                            command= lambda: controller.show_frame(StartPage))
        button1.pack()
        
        #should be title
        title = "Pressure"
        a.set_title(title)
        
        canvas = FigureCanvasTkAgg(f, self)
        canvas.draw()
        canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand = True)
        
        #this is for the navigation bar
        toolbar = NavigationToolbar2Tk(canvas, self)
        toolbar.update()
        canvas._tkcanvas.pack(side=tk.TOP, fill=tk.BOTH, expand = True)
        
        
        
app = Rocketryapp()
app.geometry("1280x720")
ani = animation.FuncAnimation(f, animate, interval=100)
app.mainloop()
    