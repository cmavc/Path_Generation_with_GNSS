"""
Creating trajectory in 2D and 3D views from GNSS log files
Developed by: Cem Avci, email: cemavci97@hotmail.com
20.12.2021

"""
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
 
excelFile=input("please enter the path of the excel file...\n")
try:
    dfList=pd.read_excel(excelFile)
    
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
        ax.set_title('Xsens MTi GNSS/INS Mapping')
        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        ax.set_zlabel('Altitude (m)')
        ax2.plot(x[0:k+200],y[0:k+200])
        ax2.set_title('Top View')
        ax3.plot(z[0:k+200])
        ax3.set_title('Altitude (m)')
        #plt.close()
        k=k+200
        #print(k)
        #plt.draw()
        plt.pause(0.001)
        ax.cla()
        ax2.cla()
        ax3.cla()


    ax.plot(x,y,z)
    ax.set_title("Xsens MTi GNSS/INS Mapping")
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax2.plot(x,y)
    ax2.set_title('Top View')
    ax3.plot(z)
    ax3.set_title('Altitude (m)')


    plt.show()


except:
    print("not a .xlsx file \nretry...")
    
