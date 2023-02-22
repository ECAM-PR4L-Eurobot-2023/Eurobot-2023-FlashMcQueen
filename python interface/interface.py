
import sys
import serial
import serial.tools.list_ports
import time
import csv
import matplotlib.pyplot as plt
from matplotlib import style
import matplotlib.animation as animation
from tkinter import *
import threading


#setup global variables
port = "undefined"          
serial_number = "18DNB803A"         #default serialnumber used to detect wich port to connect to
stopped = False


if len(sys.argv)>1:                 #if args provided at launch, change serial number 
    serial_number = sys.argv[1]

#find arduino serial port
ports= list(serial.tools.list_ports.comports())  #list all port where something is connected

print(f"Trying to connect to aarduino with serial number : {serial_number}")
for p in ports:                                 #detect wich port is connected with the device with the right serial number
    print("connected ports:")
    print(p.device)
    if p.serial_number == serial_number:
        port = str(p.device)

port = "COM3"
# setup serial communication
if port!="undefined":                       #if the right device is connected initialize serial communication
    try:
        ser = serial.Serial(port,10400)
        ser.flushInput()
        ser.flushOutput()
        ser.timeout = 10
    except Exception as e:
        print(e)
        sys.exit()

else:
    print("the arduino is not connected")
    sys.exit()



#setup plots 
fig, (ax,bx,cx) = plt.subplots(3,1)
xs = []                             #time axis used to plot data
SP = []                             #List of Setpoint over time
PV = []                             #List of Tempreature (process value) over time
MV = []                             #List of Power injected in the resistance (manipulated value) over time
AF = []                             #authorization flag 
LP = []                             #Limit in dutycycle
LPW = []                            #Limit in W
PW = []                             #list of power sent to heating resistance

def thread_window():                #Interactive window to send command to arduino via serial communication
    window = Tk()
    window.resizable(False,False)
    global input
    global Available_Power
    global max_Temperature
    global Response

    Available_Power=StringVar(window)
    labelAP = Label(window,text = "available power")

    labelAP.grid(row=0,column = 0, ipady=0,pady=0)

    Available_Power_Label = Label(window,textvariable=Available_Power)
    Available_Power_Label.grid(row=1,column = 0, ipady=0,pady=0)

    max_Temperature=StringVar(window)
    labelMT = Label(window,text = "max_Temperature")
    labelMT.grid(row=2,column = 0, ipady=0,pady=0)

    max_temperature_Label = Label(window,textvariable=max_Temperature)
    max_temperature_Label.grid(row=3,column = 0, ipady=0,pady=0)

    input = StringVar(window)
    entree = Entry(window, textvariable=input, width=30)        #fillable textblock to write command
    entree.grid(row=4,column = 0, ipady=0,pady=0)

    bouton=Button(window, text="Send", command=serial_out)      #send button to send command via serial communication
    bouton.grid(row=6,column = 0, ipady=0,pady=0)

    
    Response = StringVar(window)
    Response.set("    ")
    Response_Label = Label(window, textvariable=Response)
    Response_Label.grid(row=7,column = 0, ipady=0,pady=0)

    Quit = Button(window,text = "Quit", command=Close)
    Quit.grid(row=8, column = 0, ipady=0,pady=0)

    window.mainloop()


def serial_out():
    ser.write(bytes(input.get(),"utf-8"))       #translate utf-8 to bytes and write them on serial bus
    input.set("")                               #clears input textblock

def Close():
    global stopped
    stopped = True
    sys.exit()

def animate(i):
    """
    Needs to be called in Funcanimation for live plots of incoming data.

    Sorts the data and send it to the corresponding list.
    if all lists have the same lenght, plots the data and writes the last set of data to the csv file
    """
    if (stopped):
        sys.exit()


    data = ser.readline()               #reads next line from serial communication
    try:
        data = data.decode("utf-8")         # translate data from bytes to utf-8
    except UnicodeDecodeError:
        data=""
    print(data)
    if len(data)>0:                        #filters out unwanted serial communication
        if data[0] == "o" and data[1]=="p":                                     #next code block sorts data to the right list
            xs.append(float(data[2:]))
        if data[0] == "i" and data[1]=="p":
            if float(data[2:])<10000:
                MV.append(float(data[2:]))
                PW.append(float(data[2:])*float(Available_Power.get())/100)
            else:
                MV.append(100)
                PW.append(200)
        if data[0] == "o" and data[1]=="v":
            PV.append(float(data[2:]))
        if data[1]=="S":
            SP.append(float(data[2:]))
        if data[1]=="A":
            AF.append(float(data[2:])*50)
        if data[1]=="L":
            LP.append((float(data[2:])/float(Available_Power.get())*100))
            LPW.append((float(data[2:])))
    elif len(data)>0 and data[0]=="A" and data[1]=="P":
        Available_Power.set(data[2:])
    elif len(data)>0 and data[0]=="M" and data[1]=="T":
            pass
            max_Temperature.set(data[2:])
    else:
        Response.set(data)
    


    if True:  #checks if all the necessery data have been recieved
        ax.clear()
        bx.clear()
        cx.clear()

        if len(AF)<len(xs):                                #fills data that aren't sent every 2 seconds
            for i in range(len(xs)-len(AF)):
                AF.append(AF[-1])
        if len(AF)>len(xs):
            last = AF.pop()
            AF[-1] = last
        if len(AF)==len(xs):
            bx.step(xs,AF,label="Authorization flag")

        if len(LP)<len(xs):
            for i in range(len(xs)-len(LP)):
                LP.append(LP[-1])
                LPW.append(LPW[-1])
        if len(LP)>len(xs):
            last = LP.pop()
            LP[-1] = last
            lastW = LPW.pop()
            LPW[-1] = lastW
        if len(LP)==len(xs):
            bx.step(xs,LP,label="Limit")
            cx.step(xs,LPW, label = "Limit in watt")
        
        
        ax.set(title="Evolution of temperature and target temperature")
        ax.set(ylabel="Temperature in C°")
        bx.set(title="Evolution of dutycycle, dutycycle limit and authorization flag")
        bx.set(ylabel="DutyCycle in %")
        cx.set(title="Evolution of power and power limit")
        cx.set(ylabel="Power in Watt")

        bx.set_ylim([0,100])
        #cx.set_ylim([0,float(Available_Power.get())])

        ax.step(xs,PV,label="Temperature")
        ax.step(xs,SP,label="Target temperature")
        bx.step(xs,MV,label="Dutycycle")
        cx.step(xs,PW, label="Power")

        ax.legend(loc="best")
        bx.legend(loc="best")
        cx.legend(loc="best")
        writer.writerow((xs[-1]-xs[0],PV[-1],MV[-1],SP[-1],AF[-1],LP[-1]))    #Write data to csv file


    
thread = threading.Thread(target=thread_window)  #creates new thread with the command window (thread needed as to not stop incoming data)
thread.start()

with open(f'regulateur_{time.time()}.csv', 'w', newline='') as file:  #creates new csv file to gather data in 

    writer = csv.writer(file,delimiter = ";")
    writer.writerow(("time","Temp in C°","DutyCycle","Target Temp","Authorization flag", "P limit"))

    ani = animation.FuncAnimation(fig, animate,interval=10)
    plt.tight_layout()
    plt.show()