import tkinter as tk
from tkinter import ttk
import matplotlib
import time
matplotlib.use("TKAgg")
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
from matplotlib.figure import Figure
import matplotlib.animation as animation
from matplotlib import style
style.use('ggplot')
import random
import serial
import rvtr
from collections import deque

time.sleep(1)
f = Figure()
a = f.add_subplot(111)
try:
    arduino = serial.Serial('COM3', 115200, timeout=.1)
except Exception as e:
    print("Failed to open arduino serial port: " + str(e))
input_deque = deque([]);

def launch_rocket():
    print("Fire")

def send_command(channel, text):
    global arduino
    try:
        data = rvtr.packetify(text)
        id = channelDict[channel]
        packet = rvtr.buildPacket(id, data)
        arduino.write(bytearray(packet))
        print(channel + " (" + str(id) + "): " +
        " ".join("{:02x}".format(x) for x in packet))
    except Exception as e:
        print("Failed to send command: " + str(e))
        pass


def activateCheck1(var, Chk2, Chk3, chkvar2, chkvar3):
    if var:
        Chk2['state'] = tk.NORMAL
        print("Lock 1 Unlocked")
        send_command("/rocket/motor-unlock", "0u8")
    else:
        lockMotor(Chk2, Chk3, chkvar2, chkvar3)


def activateCheck2(var, Chk2, Chk3, chkvar2, chkvar3):
    if var:
        Chk3['state'] = tk.NORMAL
        print("Lock 2 Unlocked")
        send_command("/rocket/motor-unlock", "1u8")
    else:
        lockMotor(Chk2, Chk3, chkvar2, chkvar3)

def activateCheck3(var, Chk2, Chk3, chkvar2, chkvar3):
    if var:
        print("Lock 3 Unlocked")
        send_command("/rocket/motor-unlock", "2u8")
    else:
        lockMotor(Chk2, Chk3, chkvar2, chkvar3)

def lockMotor(Chk2, Chk3, chkvar2, chkvar3):
    Chk2['state'] = tk.DISABLED
    Chk3['state'] = tk.DISABLED
    chkvar2.set(0)
    chkvar3.set(0)
    send_command("/rocket/motor-lock", "")
    print("Motor relocked")

def Refresher():

    # TODO change this to the last n digits in the line arrary, plot graph with regresh
    # global numberPackets
    # global chanSelectVarList
    # chan = chanSelectVarList.get()
    # recivedPackets = open("./recivedpackets.txt", 'r')
    # packetArr = []
    # for line in recivedPackets:
    #     packetArr.append(line)
    #
    # xr = []
    # yr = []
    # for i in range(numberPackets):
    #     xr.append(random.randint(1,10))
    #     yr.append(random.randint(1,10))
    #
    # a.clear()
    # a.plot(xr, yr)
    # # print(xr)
    # s = ""
    # if numberPackets < 10:
    #     for i in range(numberPackets):
    #         if chan == "":
    #             s = s + packetArr[i] + "\n"
    #         else:
    #             if packetArr[i][0] == '0':
    #                 s = s + packetArr[i] + "\n"
    # else:
    #     bottom = numberPackets - 10
    #     for i in range(bottom, numberPackets):
    #         if chan == "":
    #             s = s + packetArr[i] + "\n"
    #         else:
    #             if packetArr[i][0] == '0':
    #                 s = s + packetArr[i] + "\n"

    global arduino
    while arduino.inWaiting():
        input_deque.append(int.from_bytes(arduino.read(), byteorder='big'))

    if len(input_deque) > 0:
        s = " ".join("{:02x}".format(x) for x in input_deque) + " "
        hextab.insert(tk.END, s)
    packets = rvtr.parse(input_deque)
    for packet in packets:
        print(" ".join("{:02x}".format(x) for x in packet))

    root.after(100, Refresher)


commandFile = open("./writecommand.txt", 'w')
numberPackets = 1
root = tk.Tk()
root.geometry("800x700")
topFrame = tk.Frame(root)
topFrame.pack(side='top', fill='x')

Var3 = tk.IntVar()
chk3 = tk.Checkbutton(topFrame, text='Unlock 3', variable=Var3,
state=tk.DISABLED, command=lambda:
activateCheck3(Var3.get(), chk2, chk3, Var2, Var3))
chk3.pack(side=tk.RIGHT, anchor='n')

Var2 = tk.IntVar()
chk2 = tk.Checkbutton(topFrame, text='Unlock 2', variable=Var2,
state=tk.DISABLED, command=lambda:
activateCheck2(Var2.get(), chk2, chk3, Var2, Var3))

chk2.pack(side=tk.RIGHT, anchor='n')

Var1 = tk.IntVar()
chk1 = tk.Checkbutton(topFrame, text='Unlock 1', variable=Var1,
command=lambda: activateCheck1(Var1.get(), chk2, chk3,  Var2, Var3))

chk1.pack(side=tk.RIGHT, anchor='n')

LaunchButton = tk.Button( text = "FIRE", height=100, width=100,
command=lambda: launch_rocket(), bg="red")
LaunchButton.pack(padx=10, pady=10, side=tk.RIGHT)


note = ttk.Notebook(topFrame)
note.pack(side=tk.BOTTOM, expand=True, fill='both')

tab1 = tk.Frame(note)
tab2 = tk.Frame(note)
tab3 = tk.Frame(note)
tab4 = tk.Frame(note)

# tab 1 ----------------------------------------------
channelDict = {
    "/null": 0,
    "/ground/ping": 1,
    "/rocket/ping": 2,
    "/relay/ping": 3,
    "/rocket/console": 4,
    "/relay/rocket/console": 5,
    "/relay/console": 6,
    "/rocket/status": 7,
    "/relay/rocket/status": 8,
    "/ground/fill-nitrous": 9,
    "/ground/disconnect-feed-line": 10,
    "/rocket/imudata": 11,
    "/relay/rocket/imudata": 12,
    "/ground/all-stop": 13,
    "/ground/reset": 14,
    "/ground/perform-test": 15,
    "/ground/relay-mode": 16,
    "/relay/ground/relay-mode": 17,
    "/rocket/voltage": 18,
    "/rocket/motor-info": 19,
    "/rocket/motor-unlock": 20,
    "/rocket/motor-lock": 21
}
channels = (
    "/null",
    "/ground/ping",
    "/rocket/ping",
    "/relay/ping",
    "/rocket/console",
    "/relay/rocket/console",
    "/relay/console",
    "/rocket/status",
    "/relay/rocket/status",
    "/ground/fill-nitrous",
    "/ground/disconnect-feed-line",
    "/rocket/imudata",
    "/relay/rocket/imudata",
    "/ground/all-stop",
    "/ground/reset",
    "/ground/perform-test",
    "/ground/relay-mode",
    "/relay/ground/relay-mode",
    "/rocket/voltage",
    "/rocket/motor-info",
    "/rocket/motor-unlock",
    "/rocket/motor-lock"
)

cnames = tk.StringVar(value=channels)
text_Input = tk.StringVar()
text_Output = tk.StringVar()
txtEntry = tk.Entry(tab1, textvariable=text_Input, width=50)
txtEntry.pack(side=tk.TOP, anchor='w')

chanSelectVarSend = tk.StringVar()
chanSelect = ttk.Combobox(tab1, textvariable=chanSelectVarSend)
chanSelect.pack(side=tk.RIGHT, anchor='n')
chanSelect['values'] = channels


EnterButton = tk.Button(tab1, text='Send', command=lambda:
send_command(chanSelectVarSend.get(), text_Input.get()))
EnterButton.pack(pady=4, padx=4)


display = tk.Label(tab1, text="")
display.pack(side=tk.RIGHT, anchor='s')

chanSelectVarList = tk.StringVar()
chanSelectList = ttk.Combobox(tab1, textvariable=chanSelectVarList)
chanSelectList.pack(side=tk.LEFT, anchor='s')
chanSelectList['values'] = channels

# tab2 -------------------------------------------------------


canvas = FigureCanvasTkAgg(f, tab2)
canvas.draw()
canvas.get_tk_widget().pack()

toolbar = NavigationToolbar2Tk(canvas, tab2)
toolbar.update()
canvas._tkcanvas.pack(side=tk.TOP, fill=tk.BOTH, expand=True)

# tab3 -----------------------------------
tk.Button(tab3, text='Exit', command=root.destroy).pack(padx=100, pady=100)

note.add(tab1, text="Send/Receive Commands", compound=tk.TOP)
note.add(tab2, text="Graph 1")
note.add(tab3, text="Graph 2")
note.add(tab4, text="Hexdump")
note.pack()

# tab4 -------------------------------------

hextab = tk.Text(tab4, width = 20*3);
hextab.pack();
hextab.pack()

# ani = animation.FuncAnimation(f, Refresher(), 1000)
Refresher()
root.mainloop()
