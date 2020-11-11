#!/usr/bin/env python3

import numpy as np
import cv2
from matplotlib import pyplot as plt

from ContourToWaypoints import ContourToWaypoints
from contourDetector import ContourDetector
from DriveRobot import DriveRobot


class ContourTracingBot(object):
    def __init__(self, image_path, paper_size=(5, 5), use_opencv = False):
        self.image_path = image_path
        self.raw_image = cv2.imread(image_path)
        self.gray_img = cv2.cvtColor(self.raw_image, cv2.COLOR_BGR2GRAY)
        self.img_size = (self.gray_img.shape[0], self.gray_img.shape[1])

        self.paper_size = paper_size

        self.contour = []
        self.waypoints = []
        if use_opencv:
            self.contour = self.get_contour_from_image_openCV()
        else:
            self.contour = self.get_contour_from_image_contourDetector()

    def get_contour_from_image_openCV(self):
        ret, thresh = cv2.threshold(self.gray_img, 127, 255, 0)
        contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        # cv2.drawContours(img, contours, 1, (0, 255, 0), 3)
        # cv2.imshow("Raw Image", img)
        # k = cv2.waitKey(0)
        return np.squeeze(contours[1]).tolist()

    def get_contour_from_image_contourDetector(self):
        plt.imshow(self.raw_image)
        plt.show()
        detector = ContourDetector(self.image_path)
        return detector.get_contour()

    def run(self):
        converter = ContourToWaypoints(self.contour, self.img_size, self.paper_size)
        print("Converted Contour to Waypoints")
        self.waypoints = converter.run_and_plot()
        print("waypoints: ", self.waypoints)
        print("Total number of waypoints: ", len(self.waypoints))
        bot = DriveRobot(self.waypoints)
        print("Robot Starts Driving")
        bot.run()


if __name__ == "__main__":
    img_path = "../images/shape.jpg"
    paper_dim = (5, 5)
    trace = ContourTracingBot(img_path, paper_dim, True)
    trace.run()
