import numpy as np
import cv2
from matplotlib import pyplot as plt
import math


img = cv2.imread('images/shape.jpg')
imgray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
ret, thresh = cv2.threshold(imgray, 127, 255, 0)
contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

cv2.drawContours(img, contours, 1, (0,255,0), 3)

contour_list = contours[1].tolist()

# print(type(contours[1]))


one = contours[1]
newshape = contours[1].reshape(one.shape[0],2)

x = newshape[:,0]
y = newshape[:,1]
# plt.plot(x,y)
fig = plt.figure()
ax = fig.add_subplot(1, 1, 1)
ax.scatter(x, y, s=0.1)
plt.show()

def get_distance(point_1, point_2):
  return math.sqrt(math.pow(point_2[0]-point_1[0], 2) + math.pow(point_2[1]-point_1[1], 2) * 1.0)


coordinates = []
i = 0

while i < len(x):
  coordinates.append((x[i],y[i]))
  i += 1


closest_point = (x[0], y[0])
start_point = (0,0)
closest_distance = get_distance(start_point, closest_point)

# get closest to start
for x, y in coordinates:
  temp_distance = get_distance(start_point, (x,y))
  if temp_distance < closest_distance:
    closest_point = (x,y)
    closest_distance = temp_distance

print(closest_point)







if img is None:
  sys.exit("couldn't read the image")



cv2.imshow("Display window", img)

k = cv2.waitKey(0)