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

uart_port = 'COM3'
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

sum_omega_0 = [0]

M=1
frec_ini = 0.05
T =1/10

omega_0 = [frec_ini*2*np.pi*T]

Wen = [0 , 0]

Wmos =[]
omegamos =[]

LDD_mos = []




fig = plt.figure()

ax1 = fig.add_subplot(2,2,1)
ax2 = fig.add_subplot(2,2,2)
ax3 = fig.add_subplot(2,2,3, projection='polar')
ax4 = fig.add_subplot(2,2,4)


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
    #plot_las=[]
    vector_angulos_detectados=[]
    #piernas=[]
    plot_piernas=[]
    valor_pies=[0,0]
    
    ## -- Comparacion con las curvas de trabajo
    for y in range (0,len(l)):
        if (l[y][1] > Ve[y]):
            las.append(Vi[y])
        elif (l[y][1] < Vi[y]):
            las.append(Vi[y])
        else:
            las.append(l[y][1])

    for y in range (0,len(las)):
        plot_las[y]  = (l[y][0],las[y])
        
          
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
                piernas[y] = (plot_las[y])
                #Medidas.write("%s:%s; %s \n" % (plot_las[y][0],plot_las[y][1],tiempo))
            else:
                piernas[y] = ((plot_las[y][0],0))
        valor_pies = [0,0]
    # -- En caso de que los pues esten juntos, o uno cubra el otro
    if len(vector_angulos_detectados) == 3:
        #valor_pies=[]
        
        a= (vector_angulos_detectados[1] + vector_angulos_detectados[0])//2
        b= (vector_angulos_detectados[1] + vector_angulos_detectados[2])//2
        
        for y in range (0,len (las)):
            if y ==a:
                Valor=[]
                piernas[y] = (plot_las[y])
                Medidas.write("%s:%s -" % (plot_las[y][0],plot_las[y][1]))
                Valor.append(plot_las[y][0])
                Valor.append(plot_las[y][1])
                valor_pies[0] = Valor
            if y ==b:
                Valor=[]
                piernas[y] = (plot_las[y])
                Medidas.write("%s:%s; %s \n" % (plot_las[y][0],plot_las[y][1],tiempo))
                Valor.append(plot_las[y][0])
                Valor.append(plot_las[y][1])
                valor_pies[1] = Valor
            else:
                piernas[y] = (plot_las[y][0],0)

    # -- En caso de dos pies
    if len(vector_angulos_detectados) == 4:
        #valor_pies=[]
        
        a= (vector_angulos_detectados[1] + vector_angulos_detectados[0])//2
        b= (vector_angulos_detectados[3] + vector_angulos_detectados[2])//2
        for y in range (0,len (las)):
            if y ==a:
                Valor=[]
                piernas[y] = (plot_las[y])
                Medidas.write("%s:%s - " % (plot_las[y][0],plot_las[y][1]))
                Valor.append(plot_las[y][0])
                Valor.append(plot_las[y][1])
                valor_pies[0] = Valor
            if y ==b:
                Valor=[]
                piernas[y] = (plot_las[y])
                Medidas.write("%s:%s; %s \n" % (plot_las[y][0],plot_las[y][1],tiempo))
                Valor.append(plot_las[y][0])
                Valor.append(plot_las[y][1])
                valor_pies[1] = Valor
            else:
                piernas[y] = (plot_las[y][0],0)

        

    

    for y in range (0,len(pos)):
        plot_piernas.append((pos[y][0],pos[y][1]))

    if len(vector_angulos_detectados) == 4 or len(vector_angulos_detectados) == 3:
        return plot_las,piernas,valor_pies
    else:
        for y in range (0,len (las)):
            piernas[y] = (0,0)
            
        return plot_las,piernas,[[0,0],[0,0]]
        
## -- Función para graficar de forma animada

def animate(i):
#def animate(i):
        
    #plot_las,piernas,valor_pies = Leer_laser()
    #print(valor_pies)
    Distancia_LDD = LDD(valor_pies)
    #print(Distancia_LDD)   
    Distancia_LDD_nom = Distancia_LDD/600
    
    
    #tremorsal,Xsal,Wsal,omega_0_Hzsal,sum_omega_0sal = wflc (senal, mu_0, mu_1, mu_b, frec_ini, M, sum_omega_0, W, omega_0)
    tremorsal,Xsal,Wsal,omega_0_Hzsal,omega_0sal,sum_omega_0sal = wflc (Distancia_LDD_nom, 0.15, 0.4, 0, 1, sum_omega_0[0], Wen, omega_0[0])
    #return tremor,X,Ws,omega_0_Hz,omega_0s,sum_omega_0s

    Wen[0] = Wsal[0]
    Wen[1] = Wsal[1]
    sum_omega_0[0] = sum_omega_0sal
    omega_0[0] = omega_0sal

    #print("cadencia: " + str(omega_0sal))
    #print("sum omega 0: " + str(sum_omega_0sal))
    
    if len(Wmos) == 100:
        for x in range (0,len(Wmos)):
            if x < len(Wmos)-1:
                Wmos[x] = Wmos[x+1]
                omegamos[x] = omegamos[x+1]
                LDD_mos[x] = LDD_mos[x+1]
            else:
                Wmos[x] = ( ((Wsal[0]**2) + (Wsal[1]**2))**(1/2) ) * 600
                omegamos[x] = omega_0_Hzsal
                LDD_mos[x] = Distancia_LDD 

    else:
        Wmos.append( ( ((Wsal[0]**2) + (Wsal[1]**2))**(1/2) ) * 600) 
        omegamos.append(omega_0_Hzsal)
        LDD_mos.append(Distancia_LDD)
        
        
    
    
    
    theta = np.array([x[0]*np.pi/180 for x in plot_las])
    radius = np.array([x[1] for x in plot_las])
    theta3 = np.array([x[0]*np.pi/180 for x in  piernas])
    radius3 = np.array([x[1] for x in  piernas])

    ax1.clear()
    ax1.set_title("WFLC Cadencia")
    ax1.set_ylabel("Hz")
    ax1.set_ylim(0,3)
    ax1.plot(omegamos)
    

    ax2.clear()
    ax2.set_title("FLC Amplitud")
    ax2.set_ylabel("mm")
    ax2.set_ylim(0,700)
    ax2.plot(Wmos)
    
    ax3.clear()
    ax3.set_title("Posición de las Piernas")
    ax3.plot(theta, radius, color='b', linewidth=1)
    ax3.plot(theta3, radius3, color='k', linewidth=1)
    ax3.set_rmax(2000)
    ax3.grid(True)
    

    ax4.clear()
    ax4.set_title("Señal LDD")
    ax4.set_ylabel("mm")
    ax4.set_ylim(-700,700)
    ax4.plot(LDD_mos)

    mean = 20
    ampli=0
    
    if len(Wmos) > mean: 
        for x in range (0, mean -1):
            ampli = (ampli + Wmos[len(Wmos) - 1  - x]) 
        ampli = ampli/mean * (omegamos[len(omegamos) - 1]*0.0542 + 1.0283)
    else:
        for x in range (0, len(Wmos) -1):
            ampli = ampli + Wmos[len(Wmos) - 1 - x] 
        ampli = ampli/len(Wmos) * (omegamos[len(omegamos) - 1]*0.0542 + 1.0283)
    #print("ampli: " + str(ampli))    
    vel = (omegamos[len(omegamos) - 1] * ampli/1000 *60*60)/1000
    print(vel)
    
    return 

## -- Función para La cadencia

# Periodo de actualización



def wflc (senal, mu_0, mu_1, mu_b, M, sum_omega_0, W, omega_0):

    T =1/10
    X=[]
    
    for j in range (0,M):
        X.append(math.sin((j+1)*sum_omega_0))
        X.append(math.cos((j+1)*sum_omega_0))
    #print("X0: " + str(X[0]))
    #print("X1: " + str(X[1]))
    #print("W0: " + str(W[0]))
    #print("W1: " + str(W[1]))

    MUL_XW=0
    for z in range (0,len(W)):
        MUL_XW= MUL_XW + X[z]* W[z]
        

    error = senal - MUL_XW - mu_b
    #vec_error.append(error)

 
    sum_rec = 0

    for j in range (0,M):  
        sum_rec = sum_rec + (j+1)*( W[j]*X[M+j] - W[M+j]*X[j])
        #print("sum rec: " + str(sum_rec))
        #sum_rec_vec.append(sum_rec)
   

    omega_0_pred = omega_0 + 2*mu_0*error*sum_rec

    #print("sum rec: " + str(sum_rec))
    #print("error: " + str(error))
    #print("omega_0_pred: " + str(omega_0_pred))

    W_pred = []
    for z in range(0,len(W)):
        W_pred.append(W[z] + 2*mu_1*X[z]*error)
     
    #omega_0.append(omega_0_pred)
    omega_0s = omega_0_pred

    Ws=[]
    for z in range(0,len(W_pred)):
        Ws.append(W_pred[z])
        
    sum_omega_0_ant = sum_omega_0            
    sum_omega_0s    = sum_omega_0 + omega_0_pred;
    sum_pend = abs(((sum_omega_0 - sum_omega_0_ant))*57.29578)
        

    harmonics = []
    for z in range(0,len(W)):
        harmonics.append(W[z] * X[z])

    
    suma = 0
    for z in range(0,len(harmonics)):    
        suma =  suma + harmonics[z]

    tremor = suma

    
    
    omega_0_Hz = (omega_0s/(2*np.pi)/T)
   
    
    return tremor,X,Ws,omega_0_Hz,omega_0s,sum_omega_0s

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
    
        
## -- Funcion para el hilo
def Leer_laser_cont():
    while sal == 0:
        
        plot_las1,piernas1,valor_pies1  = Leer_laser()

        for mm in range (0,len(plot_las1)):
            plot_las[mm] = plot_las1[mm]
            piernas = piernas1[mm]
            
        valor_pies[0] = valor_pies1[0]
        valor_pies[1] = valor_pies1[1]
        
    


## -- Inicio del Programa -- Inicio del Programa -- Inicio del Programa -- Inicio del Programa
## -- Inicio del Programa -- Inicio del Programa -- Inicio del Programa -- Inicio del Programa  
laser.laser_on()
 #print(laser.get_single_scan(START_STEP, STOP_STEP, 1))
dis = len(laser.get_single_scan(START_STEP, STOP_STEP, 1))

plot_las = []
piernas =[]
for x in range (0,dis):
    plot_las.append((0,0))
    piernas.append((0,0))


valor_pies = [(0,0),(0,0)]
sal = 0

thread = threading.Thread(target=Leer_laser_cont)
thread.daemon = True
thread.start()

Leer_laser()

ani= animation.FuncAnimation(fig, animate, interval=1)
try:
    # Inicializar variables
    plt.show()

except:
    #Medidas.close()
    ##### Medidas_Parametros.close()
    sal =1
    laser.laser_off()

## -- Fin del Programa -- Fin del Programa  -- Fin del Programa  -- Fin del Programa -- Fin del Programa
## -- Fin del Programa -- Fin del Programa  -- Fin del Programa  -- Fin del Programa -- Fin del Programa   

