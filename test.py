# https://stackoverflow.com/questions/46441893/connected-component-labeling-in-python/46442154

import cv2
import numpy as np

img = cv2.imread('images/shape.jpg', 0)
edges = cv2.Canny(img, 100, 200)
img = cv2.threshold(edges, 127, 255, cv2.THRESH_BINARY)[1]  # ensure binary
num_labels, labels_im = cv2.connectedComponents(img)

labels = labels_im

# Map component labels to hue val
label_hue = np.uint8(179*labels/np.max(labels))
blank_ch = 255*np.ones_like(label_hue)
labeled_img = cv2.merge([label_hue, blank_ch, blank_ch])

# cvt to BGR for display
labeled_img = cv2.cvtColor(labeled_img, cv2.COLOR_HSV2BGR)

# set bg label to black
labeled_img[label_hue==0] = 0

cv2.imshow('labeled.png', labeled_img)
cv2.waitKey()




