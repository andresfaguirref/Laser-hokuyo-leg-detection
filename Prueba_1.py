## -- Libraries  -- Libraries  -- Libraries  -- Libraries  -- Libraries  -- Libraries   
from __future__ import division

import serial
import hokuyo
import serial_port
import time
import numpy as np
from matplotlib import pyplot as plt
import matplotlib.animation as animation
import threading

import time

import random
from sklearn.cluster import KMeans

## -- Function Polar to cartesian -- Function Polar to cartesian

def polar_to_cartesian(radius_list,angles_list):
    cordenates_XY = []
    X_codenates = []
    Y_codenates = []
    for x in range (0,len(radius_list)):
        X = np.cos(angles_list[x]*np.pi/180)*radius_list[x]
        Y = np.sin(angles_list[x]*np.pi/180)*radius_list[x]
        cordenates_XY.append([X,Y])
        X_codenates.append(X)
        Y_codenates.append(Y)
    return cordenates_XY,X_codenates,Y_codenates

## Find cluster legs

def Cluster_legs():

    global Laser_lec, Clusters_legs, prev_cluster, LDD_LRF, Clusters_left_leg,Clusters_rigth_leg

    LRF_distances = []
    LRF_Angles = []

    for x in range(0,len(Laser_lec)):
        LRF_distances.append(Laser_lec[x][1])
        LRF_Angles.append(Laser_lec[x][0])

    Transitions_position = []
    Threshold = 200
    for x in range (1,len(LRF_distances)):
        b = LRF_distances[x] - LRF_distances[x-1]
        if ((b > Threshold) or (b < (-1*Threshold))):
            Transitions_position.append(x)
            
    if len(Transitions_position) == 4 and len(prev_cluster) == 0:
        points,X_p,Y_p = polar_to_cartesian(LRF_distances,LRF_Angles)
        kmeans = KMeans(n_clusters=3, random_state=0).fit(points)
        prev_cluster = []
        for y in range (0,len(kmeans.cluster_centers_)):
            magnitude = (kmeans.cluster_centers_[y][0]**2 + kmeans.cluster_centers_[y][1]**2)**(0.5)
            if magnitude > 100:
                prev_cluster.append([kmeans.cluster_centers_[y][0],kmeans.cluster_centers_[y][1]])

        if prev_cluster[0][0] > prev_cluster[1][0]:
            Clusters_rigth_leg = prev_cluster[0]
            Clusters_left_leg = prev_cluster[1]
        else:
            Clusters_rigth_leg = prev_cluster[1]
            Clusters_left_leg = prev_cluster[0]
            
        LDD_LRF.append(Clusters_rigth_leg[1] - Clusters_left_leg[1])
                
        
    elif len(prev_cluster) == 2:
        points,X_p,Y_p = polar_to_cartesian(LRF_distances,LRF_Angles)
        Threshold_cluster = 300
        new_points = []
        Threshold_mag_clster = 30

        for y in range (0,len(points)):
            distance_1 = ( (prev_cluster[0][0] - points[y][0])**2 + (prev_cluster[0][1] - points[y][1])**2 )**(0.5) 
            distance_2 = ( (prev_cluster[1][0] - points[y][0])**2 + (prev_cluster[1][1] - points[y][1])**2 )**(0.5)           
            if distance_2 < Threshold_cluster or distance_1 < Threshold_cluster:
                new_points.append(points[y])
        
        mag_cluster_1 = (prev_cluster[0][0]**2 + prev_cluster[0][1]**2)**(0.5)
        mag_cluster_2 = (prev_cluster[1][0]**2 + prev_cluster[1][1]**2)**(0.5)
        if len(new_points) > 1 and (mag_cluster_1 > Threshold_mag_clster) and (mag_cluster_2 > Threshold_mag_clster):
            kmeans = KMeans(n_clusters=2, random_state=0).fit(new_points)
            prev_cluster = []
            for z in range (0,len(kmeans.cluster_centers_)):
                prev_cluster.append([kmeans.cluster_centers_[z][0],kmeans.cluster_centers_[z][1]])
            

            if prev_cluster[0][0] > prev_cluster[1][0]:
                Clusters_rigth_leg = prev_cluster[0]
                Clusters_left_leg = prev_cluster[1]
            else:
                Clusters_rigth_leg = prev_cluster[1]
                Clusters_left_leg = prev_cluster[0]
            
            LDD_LRF.append(Clusters_rigth_leg[1] - Clusters_left_leg[1])

        else:
            prev_cluster = []
    else:
        prev_cluster = []
    

## -- Laser range finder scan object -- Laser range finder scan object -- Laser range finder scan object

class LRF(object):
    def __init__(self,uart_speed = 9800,uart_port = "COM1",star_step = 44,stop_step = 725):
      
        self.laser_serial = serial.Serial(port=uart_port, baudrate=uart_speed, timeout=0.5)
        self.port = serial_port.SerialPort(self.laser_serial)
        self.laser = hokuyo.Hokuyo(self.port)
        self.star_step = star_step
        self.stop_step = stop_step

    def laser_on(self):
        self.laser.laser_on()

    def laser_off(self):
        self.laser.laser_off()
        
    def read_one_scan(self):
        return self.laser.get_single_scan(self.star_step, self.stop_step, 1)

## -- Port LRF detection -- Port LRF detection -- Port LRF detection -- Port LRF detection 

def Find_port_URG_laser():
    import serial.tools.list_ports
    ports = list(serial.tools.list_ports.comports())
    Found = False 
    for x in ports:
        if x[1].find("URG Series") == 0:
            Laser_port = x[0]
            Found = True
            break

    if Found == True:
        return Laser_port
    else:
        print ("didn't find laser port")
        return "Error"


## -- Animate plot fuction -- Animate plot fuction -- Animate plot fuction -- Animate plot fuction
    
def animate(i):

    global Laser_lec,prev_cluster,LDD_LRF,Clusters_left_leg,Clusters_rigth_leg
    
    theta = np.array([x[0]*np.pi/180 for x in Laser_lec])
    radius = np.array([x[1] for x in Laser_lec])

    LRF_distances1 = []
    LRF_Angles1 = []

    for x in range(0,len(Laser_lec)):
        LRF_distances1.append(Laser_lec[x][1])
        LRF_Angles1.append(Laser_lec[x][0])

    points1,X_p1,Y_p1 = polar_to_cartesian(LRF_distances1,LRF_Angles1)
    
    ax0.clear()
    ax0.set_ylim(0,2000)
    ax0.set_xlim(-1000,1000)
    
    #ax.plot(theta, radius, color='b', linewidth=1)
    ax0.plot(X_p1, Y_p1, color='b', linewidth=1)

    if len(prev_cluster) == 2:
        theta1 = []
        radius1 = []
        
        for m in range(0,len(prev_cluster)):
            theta1.append(np.arctan(prev_cluster[m][1]/prev_cluster[m][0])*-180/np.pi)
            radius1.append( (prev_cluster[m][1]**2 + prev_cluster[m][0]**2)**(0.5) )
        #ax.plot(theta1, radius1, color='k', linewidth=1)
        X_cluster = [prev_cluster[0][0],prev_cluster[1][0]]
        Y_cluster = [prev_cluster[0][1],prev_cluster[1][1]]
        ax0.plot(Clusters_left_leg[0], Clusters_left_leg[1],'ko', linewidth=3)
        ax0.plot(Clusters_rigth_leg[0], Clusters_rigth_leg[1],'ro', linewidth=2)
    #ax.set_rmax(2000)
    ax0.grid(True)
    plt.title("LRF scan", va='bottom')

    ax1.clear()
    ax1.set_title("LDD signal")
    ax1.set_ylabel("(mm)")
    ax1.set_ylim(-400,400)
    ax1.plot(LDD_LRF)
    
    if len(prev_cluster) == 2:
        Medidas.write(str(Clusters_rigth_leg[0]) + "," + str(Clusters_rigth_leg[1]) + "," + str(Clusters_left_leg[0]) + "," + str(Clusters_left_leg[1]) + " \n")
    


## -- Thread for LRF Lecture

def LRF_Lecture():
    
    global Laser_lecture,Laser_obj,Laser_lec,Time_LRF

    while Laser_lecture:
        Laser_lec = Laser_obj.read_one_scan()
        Time_LRF.append(time.time())
        new_lec = []
        for x in range(0,len(Laser_lec)):
            if Laser_lec[x][1] > 2000:
                new_lec.append((Laser_lec[x][0],0))
            else:
                new_lec.append((Laser_lec[x][0],Laser_lec[x][1]))
        Laser_lec = new_lec
                
        Laser_scan_regitry.append(Laser_lec)

        Cluster_legs()
        

    print("Laser off")


# -- Global variables -- Global variables -- Global variables -- Global variables

Laser_obj = None # Object to manipulate the Hokuyo URG scan laser
Laser_lec = [] # List that contains the laser's samples
Laser_lecture = False # Variable for begin or finish the lecture of the LRF
Time_LRF = [] # List that contains the time of each sample of the LRF
Laser_scan_regitry = [] # List that each list scan

Clusters_left_leg = []
Clusters_rigth_leg = []
prev_cluster = []
LDD_LRF = [] 



## -- Program's Main -- Program's Main -- Program's Main -- Program's Main -- Program's Main
   
if __name__ == "__main__":

    ##nombre=input("Nombre del archivo para almacenar: ")
    nombre = "Prueba"
    Medidas= open(nombre + ".txt",'w')
    
    LRF_port = Find_port_URG_laser()
    start = 300
    stop = 468
    serial_comunication_speed = 256000

    if LRF_port != "Error":

        #global Laser

        # LRF object declaration
        
        Laser_obj = LRF(serial_comunication_speed,LRF_port,start,stop)
        Laser_obj.laser_on()

        # Thead LRF lecture begins
        
        Laser_lecture = True
        try:
            thread_LRF_lecture = threading.Thread(target=LRF_Lecture)
            thread_LRF_lecture.daemon = True
            thread_LRF_lecture.start()

        except:
            Medidas.close()
            Laser_obj.laser_off()
            print("thread of the LRF lecture does not work")
        
        # Initialize the figure to plot
        
        fig = plt.figure()
        #ax = fig.add_subplot(111, projection='polar')
        ax0 = fig.add_subplot(1,2,1)
        ax1 = fig.add_subplot(1,2,2)
        
        ani= animation.FuncAnimation(fig, animate, interval=20)

        fig.show()
        Laser_lecture = False
        Laser_obj.laser_off()

