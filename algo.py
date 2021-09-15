#############################
#the point of this code is to get the points in the room by file and calculate the exit using the furthest median of angle slices and return it by file
#############################
import matplotlib.pyplot as plt
import numpy as np
import csv
from mpl_toolkits.mplot3d import Axes3D
import cv2 as cv
from sklearn.cluster import DBSCAN
from sklearn import metrics
from sklearn.datasets import make_blobs
from sklearn.preprocessing import StandardScaler
import os
import time
def argmedian(x): #a function that gives the index of the median [internet]
  return np.argpartition(x, len(x) // 2)[len(x) // 2]

def distance(point1,point2): #[ours]
    return np.sqrt((point1[0]-point2[0])**2 + (point1[1]-point2[1])**2)
#[this whole code is ours]
x = 1 # the difference of each slice in degrees
y = 20 # the size of each slice in degrees
location = "/tmp/pointData.csv" # the location of the data
while not os.path.isfile(location): # wait to recieve the data from the cpp
    time.sleep(3)
room = csv.reader(open(location),delimiter =',')
room = list(room)
x_coordinates = []
y_coordinates = []
z_coordinates = []
points = []
angles = []
dists = []
for row in room: # extract the data into coordinates points angles and distances (distances to (0,0))
    x_coordinates.append(float(row[0]))
    y_coordinates.append(float(row[2]))
    z_coordinates.append(float(row[1]))
    points.append((float(row[0]),float(row[2])))
    angles.append(float(row[2])/float(row[0]))
    dists.append(distance((0,0),(float(row[0]),float(row[2]))))               
angles = np.arctan(angles) # get the angles in degrees
angles = np.rad2deg(angles)
pbyang = [] # array of arrays each array has points and the arrays are ordered by angles
dbyang = [] # array of arrays each array has distances and each distance correspondes to the point in the same location in pbyang
for i in range(int(360/x)): #create the arrays inside
    pbyang.append([])
    dbyang.append([])
for i in range(len(points)): # fill the arrays with the correct points and distances
    if angles[i]<0:
        angles[i]+=360
    angles[i]/=x
    pbyang[int(np.floor(angles[i]))].append(points[i])
    dbyang[int(np.floor(angles[i]))].append(dists[i])
sus = [] # points that are suspected to be the exit
sus_x =[] # x coordunates of points in sus
sus_y = [] # y coordunates of points in sus
for i in range(int(360/x)):
    ypoints = [] #the points in the current slice
    ydists = [] #the distances in the current slice correspondes to points in ypoints
    for j in range(y): #gets the median of each subslice
        newang = int((i+j)%(360/x))
        for k in range(len(pbyang[newang])):
            ypoints.append(pbyang[newang][k])
            ydists.append(dbyang[newang][k])
        if len(ypoints)>0:
            mediani = argmedian(ydists)
            sus.append(ypoints[mediani])
            sus_x.append(ypoints[mediani][0])
            sus_y.append(ypoints[mediani][1])
            
far = (0,0) #the furthest point
for i in range(len(sus)): #search for the furthest point
    if distance((0,0),sus[i]) > distance((0,0),far):
        far = sus[i]

fig = plt.figure()#plot the result and save for viewing
plt.scatter(x_coordinates,y_coordinates,s=5,color = 'blue')
plt.scatter(sus_x,sus_y,color = 'green')
plt.scatter([far[0]],[far[1]],color = 'yellow')
plt.scatter([0],[0],color = 'pink', s = 20)
fig.savefig('graph.png')
newfar=[far[0],0,far[1]] # save the result to a file which is then read by the cpp
with open("/tmp/Result.csv", 'w') as f:
    # create the csv writer
    writer = csv.writer(f)

    # write the result to the csv file
    writer.writerow(newfar)
