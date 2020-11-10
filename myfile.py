# https://www.thepythoncode.com/article/contour-detection-opencv-python

import numpy as np
import cv2
from matplotlib import pyplot as plt
import math


img = cv2.imread('images/shape.jpg')


# convert to RGB
image = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
# convert to grayscale
gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)

# create a binary thresholded image
_, binary = cv2.threshold(gray, 225, 255, cv2.THRESH_BINARY_INV)

print(binary.tolist())


# show it
plt.imshow(binary, cmap="gray")
plt.show()







