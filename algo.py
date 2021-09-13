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
def argmedian(x):
  return np.argpartition(x, len(x) // 2)[len(x) // 2]

def distance(point1,point2):
    return np.sqrt((point1[0]-point2[0])**2 + (point1[1]-point2[1])**2)
def line_intersection(line1, line2):
    xdiff = (line1[0][0] - line1[1][0], line2[0][0] - line2[1][0])
    ydiff = (line1[0][1] - line1[1][1], line2[0][1] - line2[1][1])

    def det(a, b):
        return a[0] * b[1] - a[1] * b[0]

    div = det(xdiff, ydiff)
    if not div == 0:

        d = (det(*line1), det(*line2))
        x = det(d, xdiff) / div
        y = det(d, ydiff) / div
        return (x, y)
    return 0
x = 1
y = 20
#example "/home/peleg/Rooms/Example1.csv" real "/tmp/pointData.csv"
location = "/tmp/pointData.csv"
while not os.path.isfile(location):
    time.sleep(3)
room = csv.reader(open(location),delimiter =',')
room = list(room)
x_coordinates = []
y_coordinates = []
z_coordinates = []
points = []
angles = []
dists = []
for row in room:
    x_coordinates.append(float(row[0]))
    y_coordinates.append(float(row[2]))
    z_coordinates.append(float(row[1]))
    points.append((float(row[0]),float(row[2])))
    angles.append(float(row[2])/float(row[0]))
    dists.append(distance((0,0),(float(row[0]),float(row[2]))))               
angles = np.arctan(angles)
angles = np.rad2deg(angles)
pbyang = []
dbyang = []
for i in range(int(360/x)):
    pbyang.append([])
    dbyang.append([])
for i in range(len(points)):
    if angles[i]<0:
        angles[i]+=360
    angles[i]/=x
    pbyang[int(np.floor(angles[i]))].append(points[i])
    dbyang[int(np.floor(angles[i]))].append(dists[i])
sus = []
sus_x =[]
sus_y = []
for i in range(int(360/x)):
    ypoints = []
    ydists = []
    for j in range(y):
        newang = int((i+j)%(360/x))
        for k in range(len(pbyang[newang])):
            ypoints.append(pbyang[newang][k])
            ydists.append(dbyang[newang][k])
        if len(ypoints)>0:
            mediani = argmedian(ydists)
            sus.append(ypoints[mediani])
            sus_x.append(ypoints[mediani][0])
            sus_y.append(ypoints[mediani][1])
            
far = (0,0)
for i in range(len(sus)):
    if distance((0,0),sus[i]) > distance((0,0),far):
        far = sus[i]

fig = plt.figure()
plt.scatter(x_coordinates,y_coordinates,s=5,color = 'blue')
plt.scatter(sus_x,sus_y,color = 'green')
plt.scatter([far[0]],[far[1]],color = 'yellow')
plt.scatter([0],[0],color = 'pink', s = 20)
fig.savefig('graph.png')
newfar=[far[0],0,far[1]]
with open("/tmp/Result.csv", 'w') as f:
    # create the csv writer
    writer = csv.writer(f)

    # write a row to the csv file
    writer.writerow(newfar)












"""
# Compute DBSCAN
centers = [[1, 1], [-1, -1], [1, -1]]
labels_true = make_blobs(n_samples=750, centers=centers, cluster_std=0.4,
                            random_state=0)
centers = [[1, 1], [-1, -1], [1, -1]]
X, labels_true = make_blobs(n_samples=750, centers=centers, cluster_std=0.4,
                            random_state=0)

X = StandardScaler().fit_transform(X)

db = DBSCAN(eps=0.3, min_samples=10).fit(X)
core_samples_mask = np.zeros_like(db.labels_, dtype=bool)
core_samples_mask[db.core_sample_indices_] = True
labels = db.labels_

# Number of clusters in labels, ignoring noise if present.
n_clusters_ = len(set(labels)) - (1 if -1 in labels else 0)
n_noise_ = list(labels).count(-1)

print('Estimated number of clusters: %d' % n_clusters_)
print('Estimated number of noise points: %d' % n_noise_)
print("Homogeneity: %0.3f" % metrics.homogeneity_score(labels_true, labels))
print("Completeness: %0.3f" % metrics.completeness_score(labels_true, labels))
print("V-measure: %0.3f" % metrics.v_measure_score(labels_true, labels))
print("Adjusted Rand Index: %0.3f"
      % metrics.adjusted_rand_score(labels_true, labels))
print("Adjusted Mutual Information: %0.3f"
      % metrics.adjusted_mutual_info_score(labels_true, labels))
print("Silhouette Coefficient: %0.3f"
      % metrics.silhouette_score(X, labels))

# #############################################################################
# Plot result

# Black removed and is used for noise instead.
unique_labels = set(labels)
colors = [plt.cm.Spectral(each)
          for each in np.linspace(0, 1, len(unique_labels))]
for k, col in zip(unique_labels, colors):
    if k == -1:
        # Black used for noise.
        col = [0, 0, 0, 1]

    class_member_mask = (labels == k)

    xy = X[class_member_mask & core_samples_mask]
    plt.plot(xy[:, 0], xy[:, 1], 'o', markerfacecolor=tuple(col),
             markeredgecolor='k', markersize=14)

    xy = X[class_member_mask & ~core_samples_mask]
    plt.plot(xy[:, 0], xy[:, 1], 'o', markerfacecolor=tuple(col),
             markeredgecolor='k', markersize=6)

plt.title('Estimated number of clusters: %d' % n_clusters_)
plt.show()
"""
"""
for point in points:
    print("a")
    closest = []
    for point2 in points:
        print("b")
        dist = distance(point,point2)
        if len(closest) < 51:
            closest.append((point2,dist))
        else:
            maxi = 0
            for i in range(len(closest)):
                if closest[maxi][1]<closest[i][1]:
                    maxi = i
            if dist< closest[maxi][1]:
                closest[maxi] = (point2,dist)
    avg = 0
    for point_dist in closest:
        avg+=point_dist[1]
    avg/=len(closest)
    if avg < 1:
        sus.append(point)
far = (0,0)
sus_x =[]
sus_y=[]
for point in sus:
    print("c")
    if distance((0,0),point) > distance((0,0),far):
        far = point
    sus_x.append(point[0])
    sus_y.append(point[1])
        
fig = plt.figure()
plt.scatter(x_coordinates,y_coordinates,s=5,color = 'blue')
plt.scatter(sus_x,sus_y,color = 'green')
plt.scatter([far[0]],[far[1]],color = 'yellow')
fig.savefig('room1.png')
"""             

"""
fig = plt.figure(frameon=False)
ax = plt.Axes(fig, [0., 0., 1., 1.])
ax.set_axis_off()
fig.add_axes(ax)
ax.scatter(x_coordinates,y_coordinates,s=5)
fig.savefig('saved_figure.png')
img = cv.imread('saved_figure.png',0)
blur = cv.medianBlur(img,15)
edges = cv.Canny(blur,200,400)
#edges=img
houghlines = cv.HoughLines(edges,1,np.pi/180,43)
lines = []
for line in houghlines:
    for rho,theta in line:
        a = np.cos(theta)
        b = np.sin(theta)
        x0 = a*rho
        y0 = b*rho
        x1 = int(x0 + 1000*(-b))
        y1 = int(y0 + 1000*(a))
        x2 = int(x0 - 1000*(-b))
        y2 = int(y0 - 1000*(a))

        cv.line(img,(x1,y1),(x2,y2),(0,0,255),2)
        cv.line(edges,(x1,y1),(x2,y2),(255,255,255),2)
        lines.append([[x1,y1],[x2,y2]])

cv.imwrite('image.jpg',img)
cv.imwrite('edges.jpg',edges)
x_coordinates = []
y_coordinates = []
for line1 in lines:
    for line2 in lines:
        if not line1 == line2:
            intersection = line_intersection(line1, line2)
            if not intersection == 0:
                x_coordinates.append(intersection[0])
                y_coordinates.append(intersection[1])
                print(intersection)
                img = cv.circle(img, (int(intersection[0]),int(intersection[1])), radius=10, color=(0, 255, 255), thickness=-1)
                edges = cv.circle(edges, (int(intersection[0]),int(intersection[1])), radius=10, color=(0, 255, 255), thickness=-1)
"""
"""
fig = plt.figure(frameon=False)
ax = plt.Axes(fig, [0., 0., 1., 1.])
ax.set_axis_off()
fig.add_axes(ax)
"""
"""
cv.imwrite('image_int.jpg',img)
cv.imwrite('edges_int.jpg',edges)
ax.scatter(x_coordinates,y_coordinates,s=5)
fig.savefig('intersections.png')
fig.show()
img = cv.imread('intersections.png',0)
cv.boundingRect(img)
cv.imwrite('rectangle.jpg',img)
"""

"""
blur = cv.medianBlur(img,25)
edges = cv.Canny(blur,200,400)
plt.subplot(121),plt.imshow(img,cmap = 'gray')
plt.title('Original Image'), plt.xticks([]), plt.yticks([])
plt.subplot(122),plt.imshow(edges,cmap = 'gray')
plt.title('Edge Image'), plt.xticks([]), plt.yticks([])
plt.show()
"""
"""
#blur = cv.blur(img,(25,25))
#blur = cv.GaussianBlur(img,(25,25),0)
blur = cv.medianBlur(img,25)
#blur = cv.bilateralFilter(img,100,75,75)
plt.subplot(121),plt.imshow(img),plt.title('Original')
plt.xticks([]), plt.yticks([])
plt.subplot(122),plt.imshow(blur),plt.title('Blurred')
plt.xticks([]), plt.yticks([])
plt.show()
"""




#fig = plt.figure(figsize=(100,100))
#ax = fig.add_subplot(111, projection='3d')
#ax.scatter3D(x_coordinates,y_coordinates,z_coordinates)
#plt.show()
