# -*- coding: utf-8 -*-
"""
Created on Wed Jan  9 19:00:35 2019

@author: Alejandro
"""
#this calls for all the functionalities
import tkinter as tk 

#this is specifying the font size and type 
ROMANFONT = ("Times New Roman", 12)
#This is defining the "StartPage"


#this is defining the class for the GUI application 
#the '()' allows this class to inhereit attirbutes from another class
class Rocketryapp(tk.Tk): 
    
    #this is a method that is going to initalize this class...everytime the applciation runs, this will pop up first 
    def __init__(self, *args, **kwargs):
        #now we are going to add what we want to initilialize
        tk.Tk.__init__(self, *args, **kwargs)
        #this is the main frame that will hold everything...the motherload 
        container = tk.Frame(self)
        #fills in the limit set and expand allows you to expand the limits 
        container.pack(side = "top", fill = "both", expand = True)
        #this is simple configuration...setting the bounds (minimum and who gets priority) 
        container.grid_rowconfigure(0, weight=1)
        container.grid_columnconfigure(0, weight=1)
        
        self.frames = {}
        
        for F in (StartPage, PageOne, PageTwo):
        
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
    
        button1 = tk.Button(self, text="Visit Page 1", 
                            command= lambda: controller.show_frame(PageOne))
        button1.pack()
        
        button2 = tk.Button(self, text="Visit Page 2", 
                            command= lambda: controller.show_frame(PageTwo))
        button2.pack()
        
class PageOne(tk.Frame):
    
    def __init__(self, parent, controller):
        tk.Frame.__init__(self, parent)
        label = tk.Label(self, text="Page One", font=ROMANFONT)
        label.pack(padx=10)
        
        button1 = tk.Button(self, text="Back to Start Page", 
                            command= lambda: controller.show_frame(StartPage))
        button1.pack()
        
        button2 = tk.Button(self, text="Page Two", 
                            command= lambda: controller.show_frame(PageTwo))
        button2.pack()

class PageTwo(tk.Frame):
    
    def __init__(self, parent, controller):
        tk.Frame.__init__(self, parent)
        label = tk.Label(self, text="Page Two", font=ROMANFONT)
        label.pack(padx=10)
        
        button1 = tk.Button(self, text="Back to Start Page", 
                            command= lambda: controller.show_frame(StartPage))
        button1.pack()

        button2 = tk.Button(self, text="Page One", 
                            command= lambda: controller.show_frame(PageOne))
        button2.pack()

app = Rocketryapp()
app.mainloop()
    
    
    
    
    
    