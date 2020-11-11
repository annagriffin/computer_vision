import numpy as np
import cv2
from matplotlib import pyplot as plt
import math


class ContourToWaypoints(object):
    def __init__(self, contour=[], img_size=(640, 480), paper_size=(5, 5)):
        self.pixel_to_meter_factor = 0.0002645833  # 1 pixel = 0.0002645833 m
        self.meter_to_pixel_factor = 3779.5275591  # 1 meter = 3779.5275591 pixels

        self.original_contour = contour # for plotting

        self.contour = contour # filtered
        self.img_size_x = img_size[0] * self.pixel_to_meter_factor  # convert pixel to meter
        self.img_size_y = img_size[1] * self.pixel_to_meter_factor  # convert pixel to meter
        self.paper_size_x = paper_size[0]
        self.paper_size_y = paper_size[1]

        self.img_aspect_ratio = img_size[0] / img_size[1]
        self.paper_aspect_ratio = paper_size[0] / paper_size[1]
        self.scale_offset = 0.8
        self.scale_factor = 1

        self.waypoints = []
        self.waypoint_distance_min = 0.4  # minimum distance between each waypoint
        self.wapoint_angle_min = 3

    def get_waypoints(self):
        return self.waypoints

    def scale_image_to_fit_paper(self):
        x_overfit = self.img_size_x > self.paper_size_x
        y_overfit = self.img_size_y > self.paper_size_y
        y_diff = abs(self.img_size_y - self.paper_size_y)
        x_diff = abs(self.img_size_x - self.paper_size_x)

        if x_overfit and y_overfit:
            if x_diff < y_diff:  # height overfit more, scale image by height to fit height on paper
                self.scale_factor = self.paper_size_y / self.img_size_y
            else:  # width overfit more, scale image by width to fit width on paper
                self.scale_factor = self.paper_size_x / self.paper_size_x
        elif x_overfit:  # only width overfit, scale image by width to fit width on paper
            self.scale_factor = self.paper_size_x / self.img_size_x
        elif y_overfit:  # only height overfit, scale image by height to fit height on paper
            self.scale_factor = self.paper_size_y / self.img_size_y
        else:  # image can be fit on paper, scale it to "FIT" the paper full
            if x_diff < y_diff:  # image width and paper width has smaller scaling room, scale by width
                self.scale_factor = self.paper_size_x / self.img_size_x
            else:
                self.scale_factor = self.paper_size_y / self.img_size_y

    def scale_contour_to_fit_paper(self):
        """
          Convert contour points in unit of pixels from an image to unit of meters and scale the image to FIT on paper
        """
        for num in range(len(self.contour)):
            self.contour[num][0] = self.contour[num][0] * self.pixel_to_meter_factor * self.scale_factor
            self.contour[num][1] = self.contour[num][1] * self.pixel_to_meter_factor * self.scale_factor

    def find_closest_start_to_origin_and_reshuffle(self):
        start = self.contour[0]
        min_distance = math.sqrt(start[0]**2 + start[1]**2)
        start_index = 0
        for i in range(1, len(self.contour)):
            contour_point = self.contour[i]
            distance = math.sqrt(contour_point[0]**2 + contour_point[1]**2)
            if distance < min_distance:
                start = contour_point
                min_distance = distance
                start_index = i
        self.contour = self.contour[start_index:] + self.contour[:start_index]

    def contour_to_waypoints(self):
        counter = 1
        self.waypoints.append(self.contour[0])
        last_added_contour_pt = self.contour[counter - 1]
        while counter < (len(self.contour)-1):
            cur_contour_pt = self.contour[counter]
            next_contour_pt = self.contour[counter + 1]
            distance_last_cur = math.sqrt(
                (cur_contour_pt[0] - last_added_contour_pt[0]) ** 2 + (cur_contour_pt[1] - last_added_contour_pt[1]) ** 2)
            distance_cur_next = math.sqrt(
                (cur_contour_pt[0] - next_contour_pt[0]) ** 2 + (cur_contour_pt[1] - next_contour_pt[1]) ** 2)
            angle_diff_last_cur = math.degrees(
                math.atan2((cur_contour_pt[1] - last_added_contour_pt[1]), (cur_contour_pt[0] - last_added_contour_pt[0])))
            angle_diff_cur_next = math.degrees(
                math.atan2((next_contour_pt[1] - cur_contour_pt[1]), (next_contour_pt[0] - cur_contour_pt[0])))

            # if above threshold is not below the threshold, we add the contour point to waypoints list
            if distance_last_cur > self.waypoint_distance_min and abs(angle_diff_last_cur) > self.wapoint_angle_min:
                self.waypoints.append(cur_contour_pt)
                last_added_contour_pt = cur_contour_pt
            counter += 1
        self.waypoints.append(self.contour[len(self.contour)-1])  # always add the last one
        self.waypoints.append(self.contour[0])  # add the first one to close the loop

    def plot_contour(self):
        contour_array = np.array(self.original_contour)
        x = contour_array[:, 0]*self.pixel_to_meter_factor
        y = contour_array[:, 1]*self.pixel_to_meter_factor
        plt.plot(x, y, '-o')
        plt.xlabel("x(m)")
        plt.ylabel("y(m)")
        plt.title("Original Contour")
        plt.show()

    def plot_waypoint(self):
        waypoint_array = np.array(self.waypoints)
        x = waypoint_array[:, 0]
        y = waypoint_array[:, 1]
        plt.plot(x, y, 'r-x')
        plt.xlabel("x(m)")
        plt.ylabel("y(m)")
        plt.title("Waypoints")
        plt.show()

    def convert_contour_to_waypoints(self):
        self.scale_image_to_fit_paper()
        self.scale_contour_to_fit_paper()
        self.find_closest_start_to_origin_and_reshuffle()
        self.contour_to_waypoints()
        return self.get_waypoints()

    def run_and_plot(self):
        waypoints = self.convert_contour_to_waypoints()
        self.plot_contour()
        self.plot_waypoint()
        return waypoints


if __name__ == "__main__":
    img = cv2.imread('../../images/shape.jpg')
    gray_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    ret, thresh = cv2.threshold(gray_img, 127, 255, 0)
    contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    # cv2.drawContours(img, contours, 1, (0, 255, 0), 3)

    contour_list = np.squeeze(contours[1]).tolist()
    img_size = (gray_img.shape[0], gray_img.shape[1])
    converter = ContourToWaypoints(contour_list, img_size)
    converter.run_and_plot()


