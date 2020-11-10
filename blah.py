import numpy as np
import cv2
from matplotlib import pyplot as plt


img = cv2.imread('images/leaf.jpg')
grey = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
invert = cv2.bitwise_not(grey)

# edges = cv2.Canny(grey, 100, 200)


ret, thresh = cv2.threshold(invert, 127, 255, 0)

# plt.subplot(121),plt.imshow(img,cmap = 'gray')
# plt.title('Original Image'), plt.xticks([]), plt.yticks([])
# plt.subplot(122),plt.imshow(edges,cmap = 'gray')
contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

cv2.drawContours(img, contours, -1, (0, 225, 0), 3)


# plt.show()

if img is None:
  sys.exit("couldn't read the image")



cv2.imshow("Display window", img)
cv2.imshow("Grey window", grey)
# cv2.imshow("Edges", edges)
k = cv2.waitKey(0)