import sys,os
import pandas as pd 
import matplotlib
matplotlib.use('QT5Agg')
import numpy as np
import matplotlib.pyplot as plt 
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.backend_bases import MouseButton


R=6371
x=[]
y=[]
 
dfList=pd.read_excel('MTi-670-8A1G6-2021-12-21-12-04.xlsx')
lat=dfList['LAT']
lon=dfList['LON']
z=dfList['ALT']
#z=z[1:]


#plt.ion()
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
#ax.set_title('')
ax2 = fig.add_subplot(337)
ax3=fig.add_subplot(339)
#ax2.set_title('2d Flight Monitoring')    
#matplotlib.interactive(True) 

n=0
        
R=6371
x=[]
y=[]

for j in range(len(lat)):
    x.append(R*np.cos(lat[j]*np.pi/180.)*np.cos(lon[j]*np.pi/180.))
    y.append(R*np.cos(lat[j]*np.pi/180.)*np.sin(lon[j]*np.pi/180.))

x=(x-x[0])*1000
y=(y-y[0])*1000


k=0

while k<len(x):
    ax.plot(x[0:k+200],y[0:k+200],z[0:k+200])
    ax.set_title('06 BOF 072 Haritalandirma - MTi-G-710')
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Rakım (m)')
    ax2.plot(x[0:k+200],y[0:k+200])
    ax2.set_title('Kuş Bakışı')
    ax3.plot(z[0:k+200])
    ax3.set_title('Yükseklik (m)')
    #plt.close()
    k=k+200
    #print(k)
    #plt.draw()
    plt.pause(0.001)
    ax.cla()
    ax2.cla()
    ax3.cla()


ax.plot(x,y,z)
ax.set_title("06 BOF 072 Haritalandirma")
ax.set_xlabel('X (m)')
ax.set_ylabel('Y (m)')
ax2.plot(x,y)
ax2.set_title('Kuş Bakışı')
ax3.plot(z)
ax3.set_title('Yükseklik (m)')


plt.show()

