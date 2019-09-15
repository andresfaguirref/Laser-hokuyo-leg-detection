from __future__ import division

import xlrd
from matplotlib import pyplot as plt
import numpy as np
import random
from sklearn.cluster import KMeans




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

 
def cluster_points(X, mu):
    clusters  = {}
    for x in X:
        bestmukey = min([(i[0], np.linalg.norm(x-mu[i[0]])) \
                    for i in enumerate(mu)], key=lambda t:t[1])[0]
        try:
            clusters[bestmukey].append(x)
        except KeyError:
            clusters[bestmukey] = [x]
    return clusters
 
def find_centers(X, K):
    # Initialize to K random centers
    oldmu = random.sample(X, K)
    mu = random.sample(X, K)
    print(oldmu)
    print(mu)
    while not has_converged(mu, oldmu):
        oldmu = mu
        # Assign all points in X to clusters
        clusters = cluster_points(X, mu)
        # Reevaluate centers
        mu = reevaluate_centers(oldmu, clusters)
    return(mu, clusters)

def reevaluate_centers(mu, clusters):
    newmu = []
    keys = sorted(clusters.keys())
    for k in keys:
        newmu.append(np.mean(clusters[k], axis = 0))
    return newmu

def has_converged(mu, oldmu):
    return (set([tuple(a) for a in mu]) == set([tuple(a) for a in oldmu]))

 
            

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

for x in range (0,len(LRF_distances)):
    for y in range (0,len(LRF_distances[x])):
        if LRF_distances[x][y] > 2000:
            LRF_distances[x][y] = 0


Threshold = 100
legs_find = False
legs = []
for x in range (0,len(LRF_distances)):
    Transitions_position = []
    for y in range (1,len(LRF_distances[x])):
        b = LRF_distances[x][y] - LRF_distances[x][y-1]
        if ((b > Threshold) or (b < (-1*Threshold))):
            Transitions_position.append(y)

    if len(Transitions_position) == 4 and legs_find == False:
        
        leg_one = (Transitions_position[0] + Transitions_position[1])//2
        leg_two = (Transitions_position[2] + Transitions_position[3])//2
        legs = [leg_one,leg_two]
        plot_LRF_legs(LRF_distances[x],LRF_Angles,legs)

        points,X_p,Y_p = polar_to_cartesian(LRF_distances[x],LRF_Angles)
        kmeans = KMeans(n_clusters=3, random_state=0).fit(points)
        X_cluster = [kmeans.cluster_centers_[0][0],kmeans.cluster_centers_[1][0],kmeans.cluster_centers_[2][0]]
        Y_cluster = [kmeans.cluster_centers_[0][1],kmeans.cluster_centers_[1][1],kmeans.cluster_centers_[2][1]]
        clusters_ini = []
        for y in range (0,len(kmeans.cluster_centers_)):
            magnitude = (kmeans.cluster_centers_[y][0]**2 + kmeans.cluster_centers_[y][1]**2)**(0.5)
            if magnitude > 100:
                clusters_ini.append([kmeans.cluster_centers_[y][0],kmeans.cluster_centers_[y][1]])

        legs_find = True
        legs.append(clusters_ini)
        fig = plt.figure()
        ax = fig.add_subplot(111)
        ax.plot(X_p, Y_p, color='b', linewidth=1)
        ax.plot(X_cluster, Y_cluster, color='k', linewidth=3)
        ax.grid(True)
        plt.title("LRF scan", va='bottom')
        plt.show()
            
            
                                    

    elif legs_find == True:

        points2,X_p2,Y_p2 = polar_to_cartesian(LRF_distances[x],LRF_Angles)
        Threshold_cluster = 300
        new_points = []


        print(clusters_ini)
        #print(points2)
        for y in range (0,len(points2)):
            distance_1 = ( (clusters_ini[0][0] - points2[y][0])**2 + (clusters_ini[0][1] - points2[y][1])**2 )**(0.5) 
            distance_2 = ( (clusters_ini[1][0] - points2[y][0])**2 + (clusters_ini[1][1] - points2[y][1])**2 )**(0.5)           
            if distance_2 < Threshold_cluster or distance_1 < Threshold_cluster:
                new_points.append(points2[y])

        print(new_points)
        kmeans2 = KMeans(n_clusters=2, random_state=0).fit(new_points)
        print(kmeans2.cluster_centers_)
        X_cluster2 = [kmeans2.cluster_centers_[0][0],kmeans2.cluster_centers_[1][0]]
        Y_cluster2 = [kmeans2.cluster_centers_[0][1],kmeans2.cluster_centers_[1][1]]

        fig = plt.figure()
        ax = fig.add_subplot(111)
        ax.plot(X_p2, Y_p2, color='b', linewidth=1)
        ax.plot(X_cluster2, Y_cluster2, color='k', linewidth=3)
        ax.grid(True)
        plt.title("LRF scan", va='bottom')
        plt.show()

        clusters_ini = []
        for z in range (0,len(kmeans2.cluster_centers_)):
            clusters_ini.append([kmeans2.cluster_centers_[z][0],kmeans2.cluster_centers_[z][1]])
        
         








clusters_ini = []
points2,X_p2,Y_p2 = polar_to_cartesian(LRF_distances[Initial_detection + 1],LRF_Angles)


fig = plt.figure()
ax = fig.add_subplot(111)
ax.plot(X_p, Y_p, color='b', linewidth=1)
ax.plot(X_p2, Y_p2, color='k', linewidth=1)
ax.grid(True)
plt.title("LRF scan", va='bottom')
plt.show()


Threshold_cluster = 300
new_points = []
for x in range (0,len(points2)):
    distance_1 = ( (clusters_ini[0][0] - points2[x][0])**2 + (clusters_ini[0][1] - points2[x][1])**2 )**(0.5) 
    distance_2 = ( (clusters_ini[1][0] - points2[x][0])**2 + (clusters_ini[1][1] - points2[x][1])**2 )**(0.5)
    if distance_2 < Threshold_cluster or distance_1 < Threshold_cluster:
        new_points.append(points2[x])

print(new_points)
kmeans2 = KMeans(n_clusters=2, random_state=0).fit(new_points)

X_cluster2 = [kmeans2.cluster_centers_[0][0],kmeans2.cluster_centers_[1][0]]
Y_cluster2 = [kmeans2.cluster_centers_[0][1],kmeans2.cluster_centers_[1][1]]


fig = plt.figure()
ax = fig.add_subplot(111)
ax.plot(X_p2, Y_p2, color='b', linewidth=1)
ax.plot(X_cluster2, Y_cluster2, color='k', linewidth=3)
ax.grid(True)
plt.title("LRF scan", va='bottom')
plt.show()
