import numpy as np
import cv2
from matplotlib import pyplot as plt
import math
from contour_point import ContourPoint

class StorageTable():

  def __init__(self, img):
       
    self.img = img

    self.binary = None
    self.convert_to_binary()
    
    self.contour_points = []
    self.prev = None


  def __getitem__(self,key):
    return self.contour_points[key]
    
  def __len__(self):
    return len(self.contour_points)

  def convert_to_binary(self):

    image = cv2.cvtColor(self.img, cv2.COLOR_BGR2RGB)
    
    # convert to grayscale
    gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)

    # create a binary thresholded image
    _, binary = cv2.threshold(gray, 225, 255, cv2.THRESH_BINARY_INV)

    self.binary = binary


  def show_image(self, img):
    plt.imshow(img, cmap="gray")
    plt.show()


  def horizontal_movement(self, point, is_left):
    
    twobytwo = self.get_2x2(point, is_left)
    
    # a orientation and h orientation
    if (is_left and twobytwo == (0, 0, 0, 255)) or (not is_left and twobytwo == (0, 255, 255, 255)):
        # positive direction
        return 1
    
    # c orientation and f orientation 
    elif (is_left and twobytwo == (0, 255, 255, 255)) or (not is_left and twobytwo == (0, 0, 0, 255)):
        # negative direction
        return -1
    else:

        # no horizontal direction
        return 0
    
      
  def get_2x2(self, point, is_left):
    i = point.x
    j = point.y
    
    # left contours
    if is_left:
        return (self.binary[i][j], self.binary[i-1][j], self.binary[i-1][j+1], self.binary[i][j+1])
    
    #right contours
    else:
        return (self.binary[i][j], self.binary[i+1][j], self.binary[i+1][j-1], self.binary[i][j-1])


  
  def get_LF_points(self):

    for j, row in enumerate(self.binary):
        points_in_row = []

        for i, pixel in reversed(list(enumerate(row))):

            # left contour
            if self.prev == 0 and pixel == 255:
                new_point = ContourPoint(i, j, 1, 0)
                new_point.H = self.horizontal_movement(new_point, False)
                
                points_in_row.append(new_point)


            # right contour
            elif self.prev == 255 and pixel == 0:
                new_point = ContourPoint(i, j, -1, 0)
                new_point.H = self.horizontal_movement(new_point, True)
                
                points_in_row.append(new_point)

            self.prev = pixel

        total_points = len(points_in_row)
        for point in points_in_row:
            point.N = point.N * total_points


        self.contour_points.extend(points_in_row)


# if __name__ == '__main__':
#   image_path = "images/shape.jpg"
#   img = cv2.imread(image_path)
  
#   table = StorageTable(img)
#   table.get_LF_points()

#   print(table.contour_points)
  
  