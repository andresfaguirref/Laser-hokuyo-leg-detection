import xlrd
from matplotlib import pyplot as plt
import numpy as np

## Fuction for legs detection

def Find_legs_in_LRF(LRF_list_distance,LRF_list_angles, Threshold):

    Transitions_position = []
    for y in range (1,len(LRF_list_distance)):
        b = LRF_list_distance[y] - LRF_list_distance[y-1]
        if ((b > Threshold) or (b < (-1*Threshold))):
            Transitions_position.append(y)

    n_Transitions = len(Transitions_position)

    if n_Transitions < 3:
        return "Not legs detection"
    elif n_Transitions == 3:
        leg_one = (Transitions_position[0] + Transitions_position[1])//2
        leg_two = (Transitions_position[1] + Transitions_position[2])//2        
        return [leg_one,leg_two]
    elif n_Transitions == 4:
        leg_one = (Transitions_position[0] + Transitions_position[1])//2
        leg_two = (Transitions_position[2] + Transitions_position[3])//2
        legs = [leg_one,leg_two]
        #plot_LRF_legs(LRF_list_distance,LRF_list_angles,legs)
        return [leg_one,leg_two]
    else:
        return "Not legs detection"
        l = 10
        legs = []
        print(Transitions_position)
        for x in range(1,n_Transitions):

            diference_angle =  Transitions_position[x] - Transitions_position[x-1]
            #diference_angle = diference_angle*(-1)
            print(diference_angle)
            
            if (diference_angle > l and len(legs)==0):
                legs.append(Transitions_position[x-1])
                legs.append(Transitions_position[x])
            elif diference_angle > l:
                legs.append(Transitions_position[x])
            if len(legs) == 4:
                break
        print(legs)
        print("-------------------------")
        
        plot_LRF_legs(LRF_list_distance,LRF_list_angles,legs)
        return legs


            
## plot LRF data and legs detection
def plot_LRF_legs(LRF_list_distance1,LRF_list_angles1,legs1):
    LRF_list_angles_deg = []
    for x in range (0,len(LRF_Angles)):
        LRF_list_angles_deg.append(LRF_list_angles1[x]*np.pi/180)

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='polar')
    theta = LRF_list_angles_deg
    radius = LRF_list_distance1
    theta1 = [LRF_list_angles_deg[legs1[0]],LRF_list_angles_deg[legs1[1]]]
    radius1 = [LRF_list_distance1[legs1[0]],LRF_list_distance1[legs1[1]]] 
    ax.clear()
    ax.plot(theta, radius, color='b', linewidth=1)
    ax.plot(theta1, radius1, color='k', linewidth=1) 
    ax.set_rmax(2000)
    ax.grid(True)
    plt.title("LRF scan", va='bottom')
    plt.show()

    
def pont_to_plane(punt,vec_nom,ppla):
    D = (-1)*(vec_nom[0]*ppla[0] + vec_nom[1]*ppla[1] + vec_nom[2]*ppla[2])
    t = (-1)*(D + vec_nom[0]*punt[0] + vec_nom[1]*punt[1] + vec_nom[2]*punt[2])/(vec_nom[0]**2 + vec_nom[1]**2 + vec_nom[2]**2)
    
    point_on_plane = [punt[0] + t*vec_nom[0], punt[1] + t*vec_nom[1], punt[2] + t*vec_nom[2]]
    
    return point_on_plane

def quat_to_one_local_axis(W,X,Y,Z,neg=0):

    x2 = X**2
    y2 = Y**2
    z2 = Z**2
    xy = X * Y
    xz = X * Z
    yz = Y * Z
    wx = W * X
    wy = W * Y
    wz = W * Z

    if neg == 1:
        return [-2.0 * (xy - wz), (-1)*(1.0 - 2.0 * (x2 + z2)), -2.0 * (yz + wx)]
    else:
        return[2.0 * (xy - wz), 1.0 - 2.0 * (x2 + z2), 2.0 * (yz + wx)]

def quat_to_two_local_axis(W,X,Y,Z,neg=0):

    x2 = X**2
    y2 = Y**2
    z2 = Z**2
    xy = X * Y
    xz = X * Z
    yz = Y * Z
    wx = W * X
    wy = W * Y
    wz = W * Z

    if neg == 0: #caso 1
        return [2.0 * (xz + wy), 2.0 * (yz - wx), 1.0 - 2.0 * (x2 + y2)]
    elif neg == 1:
        return[1.0 - 2.0 * (y2 + z2), 2.0 * (xy + wz), 2.0 * (xz - wy)]
    else:
        return[1.0 - 2.0 * (y2 + z2), 2.0 * (xy + wz), 2.0 * (xz - wy)]

def quat_to_three_local_axis(W,X,Y,Z,neg=0):

    x2 = X**2
    y2 = Y**2
    z2 = Z**2
    xy = X * Y
    xz = X * Z
    yz = Y * Z
    wx = W * X
    wy = W * Y
    wz = W * Z

    if neg == 0: #caso 1
        return [1.0 - 2.0 * (y2 + z2), 2.0 * (xy + wz), 2.0 * (xz - wy)]
    elif neg == 1:
        return[2.0 * (xz + wy), 2.0 * (yz - wx), 1.0 - 2.0 * (x2 + y2)]
    else:
        return[-2.0 * (xz + wy), -2.0 * (yz - wx), -(1.0 - 2.0 * (x2 + y2))]

def dot_product(v1,v2):
    salida = (v1[0]*v2[0] + v1[1]*v2[1] + v1[2]*v2[2])
    return salida
    

## Openn excel file that contain's the LRF and Kinect data 

Name_file = "LRF and Kinect data Prueba 1"
Excel_file =xlrd.open_workbook(Name_file + ".xls")

## LRF data extraction

sheet_zero = Excel_file.sheet_by_index(0)

LRF_distances = []
LRF_time = sheet_zero.col_values(0)
LRF_time.remove("")
LRF_time.remove(u'Time (s)')
LRF_Angles = sheet_zero.row_values(1)
LRF_Angles.remove("")
for x in range (2,sheet_zero.nrows):
    LRF_distances.append(sheet_zero.row_values(x)[1:sheet_zero.ncols])


## Kinect data extraction

sheet_one = Excel_file.sheet_by_index(1)

leg_one_kinect = []
leg_two_kinect = []
spine_base_kinect = []
spine_base_kinect_V_local_Z = []
Kinect_time = sheet_one.col_values(0)
Kinect_time = Kinect_time[3:]

for x in range (3,sheet_one.nrows):
    row = sheet_one.row_values(x)
    leg_one_kinect.append([row[99],row[100],row[101]])
    leg_two_kinect.append([row[127],row[128],row[129]])
    spine_base_kinect.append([row[1],row[2],row[3]])
    Q1 = row[4]
    Q2 = row[5]
    Q3 = row[6]
    Q4 = row[7]
    spine_base_kinect_V_local_Z.append(quat_to_two_local_axis(Q1,Q2,Q3,Q4))
                          
    

## LRF data processing -- LRF data processing -- LRF data processing -- LRF data processing

for x in range (0,len(LRF_distances)):
    for y in range (0,len(LRF_distances[x])):
        if LRF_distances[x][y] > 2000:
            LRF_distances[x][y] = 0



legs_data=[]
LRF_LDD = []
Time_detection = []
for x in range(0,len(LRF_distances)):
    detection = Find_legs_in_LRF(LRF_distances[x],LRF_Angles,100)
    if detection != "Not legs detection":
        Time_detection.append(LRF_time[x])
        legs_data.append(detection)
        distance = np.sin(LRF_Angles[detection[0]]*np.pi/180)*LRF_distances[x][detection[0]] - np.sin(LRF_Angles[detection[1]]*np.pi/180)*LRF_distances[x][detection[1]]
        LRF_LDD.append(distance)


## Kinect data processing -- Kinect data processing -- Kinect data processing -- Kinect data processing
Kinect_LDD = []
Kinect_leg_diference = []
for x in range(0,len(leg_one_kinect)):
    Kinect_leg_diference.append([leg_one_kinect[x][0] - leg_two_kinect[x][0], leg_one_kinect[x][1] - leg_two_kinect[x][1], leg_one_kinect[x][2] - leg_two_kinect[x][2]])
    #Kinect_LDD.append(-1000*dot_product(spine_base_kinect_V_local_Z[x],Kinect_leg_diference[x]))
    Kinect_LDD.append(1000*Kinect_leg_diference[x][2])


LRF_initial_sample = 75
LRF_time_plot = []
m=0
Time_LRF_plot = Time_detection[LRF_initial_sample:]
for x in range (0,len(LRF_LDD[LRF_initial_sample:])):
    LRF_time_plot.append(m)
    m = m + 0.1
    Time_LRF_plot[x] = Time_LRF_plot[x] - Time_LRF_plot[0]

Kinect_initial_sample = 285
Kinect_time_plot = []
m=0
Time_Kinect_plot = Kinect_time[Kinect_initial_sample:]
for x in range (0,len(Kinect_LDD[Kinect_initial_sample:])):
    Kinect_time_plot.append(m)
    m = m + 0.03333333333333333
    Time_Kinect_plot[x] = Time_Kinect_plot[x] - Time_Kinect_plot[0]
    

fig = plt.figure()
ax = fig.add_subplot(111)
ax.clear()
ax.plot(LRF_LDD, color='b', linewidth=1)
ax.grid(True)
plt.show()

fig = plt.figure()
ax = fig.add_subplot(111)
ax.clear()
ax.plot(Kinect_LDD, color='b', linewidth=1)
ax.grid(True)
plt.show()

fig = plt.figure()
ax = fig.add_subplot(111)
ax.clear()
ax.plot(LRF_time_plot,LRF_LDD[LRF_initial_sample:], color='k', linewidth=1)
ax.plot(Kinect_time_plot,Kinect_LDD[Kinect_initial_sample:], color='b', linewidth=1)
ax.grid(True)
plt.show()




