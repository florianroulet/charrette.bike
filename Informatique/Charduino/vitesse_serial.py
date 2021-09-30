from __future__ import division
from threading import Thread
import serial
import time
import collections
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import struct
import datetime
import pandas as pd


class serialPlot:
    def __init__(self, serialPort = '/dev/ttyUSB0', serialBaud = 38400, plotLength = 100, dataNumBytes = 4):
        self.port = serialPort
        self.baud = serialBaud
        self.plotMaxLength = plotLength
        self.dataNumBytes = dataNumBytes
        self.rawData = bytearray(dataNumBytes)
        self.dataX = collections.deque([0] * plotLength, maxlen=plotLength)
        self.dataY = collections.deque([0] * plotLength, maxlen=plotLength)
        self.dataZ = collections.deque([0] * plotLength, maxlen=plotLength)
        self.dataA = collections.deque([0] * plotLength, maxlen=plotLength)
        self.dataB = collections.deque([0] * plotLength, maxlen=plotLength)

        self.isRun = True
        self.isReceiving = False
        self.thread = None
        self.plotTimer = 0
        self.previousTimer = 0
        self.csvData = []
        self.csvData.append(["csvIter","pwm","moteurState","etat","flowingState","current","kp","capteur","timestamp","d1","d2"]);


        print('Trying to connect to: ' + str(serialPort) + ' at ' + str(serialBaud) + ' BAUD.')
        try:
            self.serialConnection = serial.Serial(serialPort, serialBaud, timeout=4)
            print('Connected to ' + str(serialPort) + ' at ' + str(serialBaud) + ' BAUD.')
        except:
            print("Failed to connect with " + str(serialPort) + ' at ' + str(serialBaud) + ' BAUD.')

    def readSerialStart(self):
        if self.thread == None:
            self.thread = Thread(target=self.backgroundThread)
            self.thread.start()
            # Block till we start receiving values
            while self.isReceiving != True:
                time.sleep(0.1)

    def getSerialData(self, frame, linesX, linesY, linesZ, linesA,linesB, lineXValueText, lineYValueText, lineZValueText, lineAValueText, lineBValueText, lineXLabel, lineYLabel, lineZLabel, lineALabel, lineBLabel, timeText):
        currentTimer = time.perf_counter()
        self.plotTimer = int((currentTimer - self.previousTimer) * 1000)     # the first reading will be erroneous
        self.previousTimer = currentTimer
        timeText.set_text('Plot Interval = ' + str(self.plotTimer) + 'ms')
        #print(type(self.rawData) )
        #print(len(self.rawData))
        #v= self.rawData.split(",")

       # if 1:
        print(len(self.rawData))
        stripped=self.rawData.strip()
        format='fffffffffff'


        try:
            csvIter,pwm,moteurState,etat,flowingState,current,kp,capteur,timestamp,d1,d2= struct.unpack(format,stripped)
            #vitesse, pwm, capteur, etat, a,b,c,d,e= struct.unpack(format,stripped)    # use 'h' for a 2 byte integer
            #print(end);
            #print(end=='0x0A')
            #print("Capteur: {:4.2f} - PWM: {:3.0f} - vitesse: {:4.2f} - etat {}".format(capteur,pwm,vitesse, etat))
            self.dataX.append(etat)    # we get the latest data point and append it to our array
            self.dataY.append(current)
            self.dataZ.append(capteur)
            self.dataA.append(pwm)
            self.dataB.append(d1)
            self.csvData.append([csvIter,pwm,moteurState,etat,flowingState,current,kp,capteur,timestamp,d1,d2]);


            linesX.set_data(range(self.plotMaxLength), self.dataX)
            linesY.set_data(range(self.plotMaxLength), self.dataY)
            linesZ.set_data(range(self.plotMaxLength), self.dataZ)
            linesA.set_data(range(self.plotMaxLength), self.dataA)
            linesB.set_data(range(self.plotMaxLength), self.dataB)

            lineXValueText.set_text('[' + lineXLabel + '] = ' + '{:3.2f}'.format(etat))
            lineYValueText.set_text('[' + lineYLabel + '] = ' + '{}'.format(current) )
            lineZValueText.set_text('[' + lineZLabel + '] = ' + '{:4.2f}'.format(capteur) )
            lineAValueText.set_text('[' + lineALabel + '] = ' + '{:3.0f}'.format(pwm) )
            lineBValueText.set_text('[' + lineBLabel + '] = ' + '{:4.2f}'.format(d1) )
        except:
            print("format espéré:")
            print(struct.calcsize(format))

            print("format eu:")
            print(len(stripped))
            print("erreur lecture")
            print(len(self.rawData))

    def backgroundThread(self):    # retrieve data
        time.sleep(1.0)  # give some buffer time for retrieving data
        self.serialConnection.reset_input_buffer()
        while (self.isRun):
            self.rawData=self.serialConnection.readline().strip().strip()
            #self.serialConnection.readinto(self.rawData)
            self.isReceiving = True
            print(self.rawData)

    def close(self):
        self.isRun = False
        self.thread.join()
        self.serialConnection.close()
        print('Disconnected...')
        df = pd.DataFrame(self.csvData)
        dfName = datetime.datetime.now().strftime("%Y%m%d-%Hh%Mm%Ss")
        df.to_csv('./'+dfName+'.csv')


def main():
    # portName = 'COM5'     # for windows users
    portName = '/dev/ttyUSB0'
    baudRate = 9600
    maxPlotLength =500
    dataNumBytes =  36     # number of bytes of 1 data point
    s = serialPlot(portName, baudRate, maxPlotLength, dataNumBytes)   # initializes all required variables
    s.readSerialStart()                                               # starts background thread

    # plotting starts below
    pltInterval = 50    # Period at which the plot animation updates [ms]
    xmin = 0
    xmax = maxPlotLength
    ymin = 0
    ymax = 22
    fig = plt.figure()
    ax = plt.axes(xlim=(xmin, xmax), ylim=(float(ymin - (ymax - ymin) / 10), float(ymax + (ymax - ymin) / 10)))
    ax.set_title('Arduino Analog Read')
    ax.set_xlabel("time")
    ax.set_ylabel("AnalogRead Value")

    ax2= ax.twinx()

    lineXLabel = 'Etat'
    lineYLabel = 'Courant'
    lineZLabel = 'Capteur'
    lineALabel = 'PWM'
    lineBLabel = 'Dérivée'

    timeText = ax.text(0.50, 0.95, '', transform=ax.transAxes)
    linesX = ax.plot([], [], label=lineXLabel)[0]
    linesY = ax.plot([], [], label=lineYLabel)[0]
    linesZ = ax.plot([], [], label=lineZLabel)[0]
    linesB = ax.plot([], [], label=lineBLabel)[0]
    linesA = ax2.plot([], [], label=lineALabel,color='red')[0]
    lineXValueText = ax.text(0.50, 0.90, '', transform=ax.transAxes)
    lineYValueText = ax.text(0.50, 0.85, '', transform=ax.transAxes)
    lineZValueText = ax.text(0.50, 0.80, '', transform=ax.transAxes)
    lineAValueText = ax.text(0.50, 0.75, '', transform=ax.transAxes)
    lineBValueText = ax.text(0.50, 0.70, '', transform=ax.transAxes)

    anim = animation.FuncAnimation(fig, s.getSerialData, fargs=(linesX, linesY, linesZ, linesA,  linesB, lineXValueText, lineYValueText, lineZValueText, lineAValueText, lineBValueText, lineXLabel, lineYLabel, lineZLabel,lineALabel,lineBLabel, timeText), interval=pltInterval)    # fargs has to be a tuple

    ax2.set_ylim([80,350])
    ax2.legend(["pwm"],loc="upper left")
    ax.legend(["Etat","Courant","capteur","dérivée"],loc="upper right")
    plt.grid(which='major')
    plt.minorticks_on()
    plt.grid(which='minor',color="#555555",linestyle='-',alpha=0.5)


    plt.show()

    s.close()


if __name__ == '__main__':
    main()
