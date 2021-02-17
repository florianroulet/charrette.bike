import matplotlib.pyplot as plt
import csv
import sys
import serial
import matplotlib.animation as animation
import time
from csv import reader
import struct


time=[]
pwm=[]
moteurState=[]
capteur=[]
vitesse=[]
courant=[]
isFlowing=[]

csvDoc=sys.argv[1]
print(str(sys.argv[1]))
with open(csvDoc, 'r') as csvfile:
    plots= csv.reader(csvfile, delimiter=',')
    try:
        for row in plots:
            time.append(row[0])
            pwm.append(float(row[2]))
            moteurState.append(int(row[3]))
            capteur.append(float(row[15]))
            vitesse.append(float(row[1]))
            isFlowing.append(int(row[6]))
            courant.append(float(row[9]))
    except:
        print("erreur lecture")


fig, ax1 = plt.subplots()

color = 'tab:red'
ax1.plot(time,moteurState,time,capteur,time,vitesse,time,isFlowing,time,courant)
ax1.legend(['moteur state','capteur','vitesse', 'isFlowing','courant'])
plt.title('Remorque - '+sys.argv[1])
ax1.set_ylim([-10,30])
plt.tick_params(bottom=False,
                labelbottom=False)
ax2 = ax1.twinx()
ax2.plot(time,pwm)
ax2.legend(["PWM"])
ax2.set_ylim([80,255])


plt.xlabel('Temps')

plt.show()
