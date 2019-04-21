# -*- coding: utf-8 -*-
"""
Created on Fri Jan 11 11:59:48 2019

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

#this is specifying the font size and type 
ROMANFONT = ("Times New Roman", 12)

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

        RocketFUNC = tk.Menu(menubar, tearoff=1)
        RocketFUNC.add_command(label = "ARM",
                               command=lambda: FUNC("ARM", "arm"))
        RocketFUNC.add_command(label = "LOAD DATA",
                               command=lambda: FUNC("LOAD DATA", "load data"))
        RocketFUNC.add_command(label = "FIRE",
                               command=lambda: FUNC("FIRE", "fire"))
        RocketFUNC.add_command(label = "STIMULATE",
                               command=lambda: FUNC("STIMULATE", "stimulate"))
        RocketFUNC.add_command(label = "STOP",
                               command=lambda: FUNC("STOP", "stop"))
        RocketFUNC.add_command(label = "SAVE",
                               command=lambda: FUNC("SAVE", "save"))
        menubar.add_cascade(label="Rocket functions", menu=RocketFUNC)

        tk.Tk.config(self, menu=menubar)
        
        
#--------------------------------------SCROLLBAR-------------------------------
        
        
                
        
#------------------------------------------------------------------------------        
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
        label = tk.Label(self, text="Start Page", font=ROMANFONT)
        label.pack(padx=10)
    
        button1 = ttk.Button(self, text="Visit Page 1", 
                            command= lambda: controller.show_frame(PageOne))
        button1.pack()
        
        button2 = ttk.Button(self, text="Visit Norm Graph", 
                            command= lambda: controller.show_frame(PageTwo))
        button2.pack()
        
        button3 = ttk.Button(self, text="Visit Live Graph", 
                            command= lambda: controller.show_frame(PageThree))
        button3.pack()

        
class PageOne(tk.Frame):
    
    def __init__(self, parent, controller):
        tk.Frame.__init__(self, parent)
        label = tk.Label(self, text="Page One", font=ROMANFONT)
        label.pack(padx=10)
        
        button1 = ttk.Button(self, text="Back to Start Page", 
                            command= lambda: controller.show_frame(StartPage))
        button1.pack()
        
        button2 = ttk.Button(self, text="Norm Graph", 
                            command= lambda: controller.show_frame(PageTwo))
        button2.pack()
        
        button3 = ttk.Button(self, text="Live Graph", 
                            command= lambda: controller.show_frame(PageThree))
        button3.pack()

class PageTwo(tk.Frame):
    
    def __init__(self, parent, controller):
        tk.Frame.__init__(self, parent)
        label = tk.Label(self, text="Norm Graph", font=ROMANFONT)
        label.pack(padx=10)
        
        button1 = ttk.Button(self, text="Back to Start Page", 
                            command= lambda: controller.show_frame(StartPage))
        button1.pack()

        button2 = ttk.Button(self, text="Page One", 
                            command= lambda: controller.show_frame(PageOne))
        button2.pack()
        
        button3 = ttk.Button(self, text="Live Graph", 
                            command= lambda: controller.show_frame(PageThree))
        button3.pack()
        
        #adding the normal graph for display
        f = Figure()
        a = f.add_subplot(211)
        a.plot([1,2,3,4,5,6], [2,5,4,3,8,9])
        
        g = Figure()
        b = g.add_subplot(212)
        b.plot([2,1,3,5,4,6], [2,5,4,3,8,9])
        
        canvas = FigureCanvasTkAgg(g, self)
        canvas.draw()
        canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand = True)
        
        canvas = FigureCanvasTkAgg(f, self)
        canvas.draw()
        canvas.get_tk_widget().pack(side=tk.BOTTOM, fill=tk.BOTH, expand = True)
        
        toolbar = NavigationToolbar2Tk(canvas, self)
        toolbar.update()
        canvas._tkcanvas.pack(side=tk.BOTTOM, fill=tk.BOTH, expand = True)

        
#this page is to have a graph as an example 
class PageThree(tk.Frame):
    
    def __init__(self, parent, controller):
        tk.Frame.__init__(self, parent)
        label = tk.Label(self, text="Live Graph", font=ROMANFONT)
        label.pack(padx=10)
        
        button1 = ttk.Button(self, text="Back to Start Page", 
                            command= lambda: controller.show_frame(StartPage))
        button1.pack()


        
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
    
    
    