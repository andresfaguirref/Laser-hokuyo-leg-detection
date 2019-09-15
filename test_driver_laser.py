## -- Librerias -- Librerias -- Librerias -- Librerias -- Librerias -- Librerias  

import serial
import hokuyo
import serial_port
import time
import numpy as np
from matplotlib import pyplot as plt
import matplotlib.animation as animation
import threading

## -- Parametros de puerto (Comunicaci髇 Serial)

uart_port = 'COM7'
uart_speed = 256000
#uart_speed = 19200

## -- Parametros del Sensor (Angulos toma de muestras)

#START_STEP = 44
#STOP_STEP = 725

#START_STEP = 250
#STOP_STEP = 518

#START_STEP = 260
#STOP_STEP = 508

START_STEP = 280
STOP_STEP = 488

## -- Declaraciones para graficar 
fig = plt.figure()
ax = fig.add_subplot(111, projection='polar')

## -- Parametros de medicion (Umbral de trancicion(mm) y Curvas de trabajo)

Archivo = open("Vector_Z.txt",'r')
vector = Archivo.readlines()
Ve=[]
Vi=[]
Angulo=[]
for x in range (0,len(vector)) :  
    vector[x]=vector[x].replace("\n","")
    Int= vector[x].split(":")
    for y in range (0,len(Int)):
        Int[y]=float(Int[y])
    Angulo.append(Int[0])
    Ve.append(Int[1])
    Vi.append(Int[2])
Archivo.close()

umbral=130

## -- Archivo de texto para almacenamiento

nombre=input("Nombre del archivo para almacenar: ")
Medidas= open(nombre + ".txt",'w')
#time.sleep(10)
## -- Conexion del puerto con el laser 
    
laser_serial = serial.Serial(port=uart_port, baudrate=uart_speed, timeout=0.5)
port = serial_port.SerialPort(laser_serial)
laser = hokuyo.Hokuyo(port)

## -- Funci贸n para leer el laser y obtener la pocici贸n de las piernas

def Leer_laser():
    
    TM=0
    ## -- Toma de muestras y Declaracion de vectores 
    l=laser.get_single_scan(START_STEP, STOP_STEP, 1)
    tiempo = time.time()
    las=[]
    pos=[]
    plot_las=[]
    vector_angulos_detectados=[]
    piernas=[]
    plot_piernas=[]
    
    ## -- Comparacion con las curvas de trabajo
    for y in range (0,len(l)):
        if (l[y][1] > Ve[y]):
            las.append(Vi[y])
        elif (l[y][1] < Vi[y]):
            las.append(Vi[y])
        else:
            las.append(l[y][1])

    for y in range (0,len(las)):
        plot_las.append((l[y][0],las[y]))
            
    ## -- Detecci贸n de Tranciciones (Umbral)
    m=0
    for y in range (1,len(las)):
        entrar=[]
        b= las[y] - las[y-1]
        if ((b > umbral) or (b < (-1*umbral))):
            m=m+1
            entrar.append(l[y][0])
            entrar.append(plot_las[y][1])
            vector_angulos_detectados.append(y)
        else:
            entrar.append(l[y][0])
            entrar.append(0)
        pos.append(entrar)

    
    ## -- Almacenamiento segun la cantidad de tranciciones 
    # -- En caso de 1 solo pie
    if len(vector_angulos_detectados) == 2:
        a= (vector_angulos_detectados[1] + vector_angulos_detectados[0])//2
        
        for y in range (0,len (las)):
            if y ==a:
                piernas.append(plot_las[y])
                Medidas.write("%s:%s; %s \n" % (plot_las[y][0],plot_las[y][1],tiempo))
            else:
                piernas.append((plot_las[y][0],0))
    # -- En caso de que los pues esten juntos, o uno cubra el otro
    if len(vector_angulos_detectados) == 3:
        a= (vector_angulos_detectados[1] + vector_angulos_detectados[0])//2
        b= (vector_angulos_detectados[1] + vector_angulos_detectados[2])//2
        
        for y in range (0,len (las)):
            if y ==a:
                piernas.append(plot_las[y])
                Medidas.write("%s:%s -" % (plot_las[y][0],plot_las[y][1]))
            if y ==b:
                piernas.append(plot_las[y])
                Medidas.write("%s:%s; %s \n" % (plot_las[y][0],plot_las[y][1],tiempo))
            else:
                piernas.append((plot_las[y][0],0))

    # -- En caso de dos pies
    if len(vector_angulos_detectados) == 4:
        a= (vector_angulos_detectados[1] + vector_angulos_detectados[0])//2
        b= (vector_angulos_detectados[3] + vector_angulos_detectados[2])//2
        for y in range (0,len (las)):
            if y ==a:
                piernas.append(plot_las[y])
                Medidas.write("%s:%s - " % (plot_las[y][0],plot_las[y][1]))
            if y ==b:
                piernas.append(plot_las[y])
                Medidas.write("%s:%s; %s \n" % (plot_las[y][0],plot_las[y][1],tiempo))
            else:
                piernas.append((plot_las[y][0],0))

    

    for y in range (0,len(pos)):
        plot_piernas.append((pos[y][0],pos[y][1]))

    return plot_las,piernas

## -- Funci贸n para graficar de forma animada

def animate(i):
    
    plot_las,piernas = Leer_laser()
    theta = np.array([x[0]*np.pi/180 for x in plot_las])
    radius = np.array([x[1] for x in plot_las])
    theta3 = np.array([x[0]*np.pi/180 for x in  piernas])
    radius3 = np.array([x[1] for x in  piernas])  
    ax.clear()
    ax.set_rmax(1000)
    ax.plot(theta, radius, color='b', linewidth=1)
    ax.plot(theta3, radius3, color='k', linewidth=1)
    ax.set_rmax(2000)
    ax.grid(True)
    plt.title("Grafico Polar", va='bottom')


## -- Inicio del Programa -- Inicio del Programa -- Inicio del Programa -- Inicio del Programa
## -- Inicio del Programa -- Inicio del Programa -- Inicio del Programa -- Inicio del Programa  
laser.laser_on()
print(laser.get_single_scan(START_STEP, STOP_STEP, 1))
print(len(laser.get_single_scan(START_STEP, STOP_STEP, 1)))

ani= animation.FuncAnimation(fig, animate, interval=1)

try:
    plt.show()
except:
    Medidas.close()
    laser.laser_off()

## -- Fin del Programa -- Fin del Programa  -- Fin del Programa  -- Fin del Programa -- Fin del Programa
## -- Fin del Programa -- Fin del Programa  -- Fin del Programa  -- Fin del Programa -- Fin del Programa   
