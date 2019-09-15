## -- Librerias -- Librerias -- Librerias -- Librerias -- Librerias -- Librerias  

import serial
import hokuyo
import serial_port
import time
import numpy as np
import math
from matplotlib import pyplot as plt
import matplotlib.animation as animation
import threading

## --  Funcinoes --  Funcinoes --  Funcinoes



## -- Parametros de puerto (Comunicación Serial)

uart_port = 'COM6'
uart_speed = 256000
#uart_speed = 19200

## -- Parametros del Sensor (Angulos toma de muestras)

#START_STEP = 44
#STOP_STEP = 725

#START_STEP = 250
#STOP_STEP = 518

#START_STEP = 260
#STOP_STEP = 508

#START_STEP = 280
#STOP_STEP = 488

START_STEP = 300
STOP_STEP = 468

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

umbral=200

## -- Archivo de texto para almacenamiento

nombre=input("Nombre del archivo para almacenar: ")
Medidas= open(nombre + ".txt",'w')
#####Medidas_Parametros= open(nombre + "_Parametros" + ".txt",'w')
time.sleep(2)
## -- Conexion del puerto con el laser 
    
laser_serial = serial.Serial(port=uart_port, baudrate=uart_speed, timeout=0.5)
port = serial_port.SerialPort(laser_serial)
laser = hokuyo.Hokuyo(port)

## -- Función para leer el laser y obtener la pocición de las piernas

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
            
    ## -- Detección de Tranciciones (Umbral)
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
                #Medidas.write("%s:%s; %s \n" % (plot_las[y][0],plot_las[y][1],tiempo))
            else:
                piernas.append((plot_las[y][0],0))
    # -- En caso de que los pues esten juntos, o uno cubra el otro
    if len(vector_angulos_detectados) == 3:
        valor_pies=[]
        
        a= (vector_angulos_detectados[1] + vector_angulos_detectados[0])//2
        b= (vector_angulos_detectados[1] + vector_angulos_detectados[2])//2
        
        for y in range (0,len (las)):
            if y ==a:
                Valor=[]
                piernas.append(plot_las[y])
                Medidas.write("%s:%s -" % (plot_las[y][0],plot_las[y][1]))
                Valor.append(plot_las[y][0])
                Valor.append(plot_las[y][1])
                valor_pies.append(Valor)
            if y ==b:
                Valor=[]
                piernas.append(plot_las[y])
                Medidas.write("%s:%s; %s \n" % (plot_las[y][0],plot_las[y][1],tiempo))
                Valor.append(plot_las[y][0])
                Valor.append(plot_las[y][1])
                valor_pies.append(Valor)
            else:
                piernas.append((plot_las[y][0],0))

    # -- En caso de dos pies
    if len(vector_angulos_detectados) == 4:
        valor_pies=[]
        
        a= (vector_angulos_detectados[1] + vector_angulos_detectados[0])//2
        b= (vector_angulos_detectados[3] + vector_angulos_detectados[2])//2
        for y in range (0,len (las)):
            if y ==a:
                Valor=[]
                piernas.append(plot_las[y])
                Medidas.write("%s:%s - " % (plot_las[y][0],plot_las[y][1]))
                Valor.append(plot_las[y][0])
                Valor.append(plot_las[y][1])
                valor_pies.append(Valor)
            if y ==b:
                Valor=[]
                piernas.append(plot_las[y])
                Medidas.write("%s:%s; %s \n" % (plot_las[y][0],plot_las[y][1],tiempo))
                Valor.append(plot_las[y][0])
                Valor.append(plot_las[y][1])
                valor_pies.append(Valor)
            else:
                piernas.append((plot_las[y][0],0))

    

    for y in range (0,len(pos)):
        plot_piernas.append((pos[y][0],pos[y][1]))

    if len(vector_angulos_detectados) == 4 or len(vector_angulos_detectados) == 3:
        return plot_las,piernas,valor_pies
    else:
        valor_pies=[]
        return plot_las,piernas,valor_pies
        
## -- Función para graficar de forma animada

def animate(i,sum_omega_0=0,W=[0,0],omega_0=0.05*2*np.pi*(1/10),contador=0):
#def animate(i):
        
    plot_las,piernas,valor_pies = Leer_laser()
    #print(valor_pies)
    Distancia_LDD = LDD(valor_pies)
    #print(Distancia_LDD)   
    Distancia_LDD_nom=Distancia_LDD/550
    
    #####if i==0:
        
        #####tremor,X_1,W_1,omega_0_1,sum_pend_1,sum_omega_0_1,omega_0_Hz_1 = wflc (Distancia_LDD_nom, 0.015, 0.2, 0, 0.5, 1, sum_omega_0, Vec_omega_0_Hz[1], Vec_omega_0_Hz[0])     
 
#####    elif i==1:
        #####tremor,X_1,W_1,omega_0_1,sum_pend_1,sum_omega_0_1,omega_0_Hz_1 = wflc (Distancia_LDD_nom, 0.015, 0.2, 0, 0.5, 1, sum_omega_0, Vec_omega_0_Hz[3], Vec_omega_0_Hz[2])
    
  #####  else:
       ##### tremor,X_1,W_1,omega_0_1,sum_pend_1,sum_omega_0_1,omega_0_Hz_1 = wflc (Distancia_LDD_nom, 0.015, 0.2, 0, 0.5, 1, sum_omega_0, Vec_omega_0_Hz[i*2 + 1], Vec_omega_0_Hz[i*2])
        
    #tremor,X_1,W_1,omega_0_1,sum_pend_1,sum_omega_0_1,omega_0_Hz_1 = wflc (Distancia_LDD, 0.015, 0.2, 0, 0.5, 1, Vec[0], Vec[1], Vec[2])
        

    #print(Vec_omega_0_Hz)
    
    #print(len(Vec_omega_0_Hz))
    

    #Vec_omega_0_Hz=[]
    #####Vec_omega_0_Hz.append(omega_0_Hz_1)
    #####Vec_omega_0_Hz.append(W_1)
    
    
    
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
    #contador=contador+1

    #####contador= contador + 1
    return ##### contador

## -- Función para La cadencia

# Periodo de actualización



def wflc (senal, mu_0, mu_1, mu_b, frec_ini, M, sum_omega_0, W, omega_0):

    T =1/10
    X=[]
    
    for j in range (0,M):
        X.append(math.sin((j+1)*sum_omega_0))
        X.append(math.cos((j+1)*sum_omega_0))


    MUL_XW=0
    for z in range (0,len(W)):
        MUL_XW= MUL_XW + X[z]* W[z]
        

    error = senal - MUL_XW - mu_b
    #vec_error.append(error)
    
 
    sum_rec = 0

    for j in range (0,M):  
        sum_rec = sum_rec + (j+1)*(W[j]*X[M+j] - W[M+j]*X[j])
        #sum_rec_vec.append(sum_rec)


    omega_0_pred = omega_0 + 2*mu_0*error*sum_rec

    W_pred = []
    for z in range(0,len(W)):
        W_pred.append(W[z] + 2*mu_1*X[z]*error)
     
    #omega_0.append(omega_0_pred)
    omega_0 = omega_0_pred

    W=[]
    for z in range(0,len(W_pred)):
        W.append(W_pred[z])
        
    sum_omega_0_ant = sum_omega_0            
    sum_omega_0    = sum_omega_0 + omega_0_pred;
    sum_pend = abs(((sum_omega_0 - sum_omega_0_ant))*57.29578)
        

    harmonics = []
    for z in range(0,len(W)):
        harmonics.append(W[z] * X[z])

    
    suma = 0
    for z in range(0,len(harmonics)):    
        suma =  suma + harmonics[z]

    tremor = suma

    omega_0_Hz=[]
    
    omega_0_Hz = (omega_0/(2*np.pi)/T)
    #####Medidas_Parametros.write("%s \n" % (omega_0_Hz))    
    
    return tremor,X,W,omega_0,sum_pend,sum_omega_0,omega_0_Hz

## -- Función para La Distancia entre piernas (LDD)
def LDD (piernas):
    if len(piernas)==0:
        return 0
    else:     
        angulo1 = (np.pi)*(piernas[0][0])/180
        angulo2 = (np.pi)*float(piernas[1][0])/180
        distancia1= math.sin(angulo1) * (piernas[0][1])
        distancia2= math.sin(angulo2) * (piernas[1][1])
        distancia = distancia1 - distancia2
        return distancia
    
        
        


## -- Inicio del Programa -- Inicio del Programa -- Inicio del Programa -- Inicio del Programa
## -- Inicio del Programa -- Inicio del Programa -- Inicio del Programa -- Inicio del Programa  
laser.laser_on()
#print(laser.get_single_scan(START_STEP, STOP_STEP, 1))
#print(len(laser.get_single_scan(START_STEP, STOP_STEP, 1)))

ani= animation.FuncAnimation(fig, animate, interval=1)
#animate(i,sum_omega_0=0,W=[0,0],omega_0=0.05*2*np.pi*(1/10),contador=0)


W=[]
omega_0=[]
Vec=[]
vec_error=[]
sum_rec_vec=[]
sum_vec=[]
sum_pend=0
frec_ini = 0.05
sum_omega_0=0

M=1
frec_ini=0.5
T =1/10

omega_0 = frec_ini*2*np.pi*T

W=[]
vec_tremor=[]
Vec_sem_pen=[]
Vec_W=[]
Vec_X=[]
Vec_omega_0_Hz=[]
Vec_omega_0_Hz.append(0.05*2*np.pi*(1/10))
Vec=[]

for z in range (0,M*2):
    W.append(0)

Vec_omega_0_Hz.append(W)

try:
    # Inicializar variables
    a = plt.show()

except:
    Medidas.close()
    ##### Medidas_Parametros.close()
    laser.laser_off()

## -- Fin del Programa -- Fin del Programa  -- Fin del Programa  -- Fin del Programa -- Fin del Programa
## -- Fin del Programa -- Fin del Programa  -- Fin del Programa  -- Fin del Programa -- Fin del Programa   


