import matplotlib.pyplot as plt
import csv
import sys

time=[]
pwm=[]
moteurState=[]
capteur=[]
vitesseC=[]
accelC=[]

csvDoc=sys.argv[1]
print(str(sys.argv[1]))
with open(csvDoc, 'r') as csvfile:
    plots= csv.reader(csvfile, delimiter=',')
    for row in plots:
        time.append(int(row[1]))
        pwm.append(float(row[2]))
        moteurState.append(int(row[3]))
        capteur.append(float(row[12]))
        vitesseC.append(float(row[15]))
        accelC.append(float(row[16]))

fig, ax1 = plt.subplots()

color = 'tab:red'
ax1.plot(time,moteurState,time,capteur,time,vitesseC,time,accelC)
ax1.legend(['moteur state','capteur','dérivée 1', 'dérivée 2'])
plt.title('Remorque')
ax1.set_ylim([-10,10])

ax2 = ax1.twinx()
ax2.plot(time,pwm)
ax2.legend(["PWM"])
ax2.set_ylim([80,255])


plt.xlabel('Temps')

plt.show()
