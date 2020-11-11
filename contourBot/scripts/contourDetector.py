#!/usr/bin/env python3
import cv2
import math
from storageTable import StorageTable


class ContourDetector(object):
    def __init__(self, img_path=None):

        self.img = cv2.imread(img_path)
        self.storage = StorageTable(self.img)
        self.storage.get_LF_points()

        self.m = None
        self.K = None
        self.k = None

        self.points_ordered = []

    def __len__(self):
        return len(self.points_ordered)

    def filter_contour_candidates(self, current_point):
        horizontal_direction = current_point.H

        for p in reversed(self.K):
            temp_point = self.storage[p]
            if current_point.x - temp_point.x > 0 and horizontal_direction == 1:
                self.K.remove(p)
            elif current_point.x - temp_point.x < 0 and horizontal_direction == -1:
                self.K.remove(p)

        return self.get_shortest_distance(current_point)

    def get_distance(self, p1, p2):
        return math.sqrt(math.pow(p2.x - p1.x, 2) + math.pow(p2.y - p1.y, 2))

    def get_shortest_distance(self, point):

        closest_distance = None
        closest_point = None

        for each in self.K:
            dist = self.get_distance(point, self.storage[each])
            if closest_distance == None or dist < closest_distance:
                closest_distance = dist
                closest_point = each

        # index
        return closest_point

    def get_possible_contour_neighbors(self, point_idx, m):
        i = point_idx
        possible_contours = [i - 1, i + 1]
        current_point = self.storage[i]

        counter = i + current_point.N

        while counter <= point_idx + m:
            possible_contours.append(counter)
            counter += 2

        h = [n % len(self.storage) for n in possible_contours]
        return h

    def get_adjacent_scan_line_N(self, point_idx):
        current_point = self.storage[point_idx]
        isLeft = 1

        if current_point.N < 0:
            isLeft = -1

        while True:
            new_idx = point_idx + (2 * isLeft)

            next_point = self.storage[new_idx % len(self.storage)]
            if next_point.y != current_point.y:
                return next_point.N
            else:
                point_idx = new_idx
                current_point = next_point

    def reorder(self):
        i = 0
        while True:
            self.m = self.get_adjacent_scan_line_N(i)
            self.K = self.get_possible_contour_neighbors(i, self.m)
            self.k = self.K[0]
            # self.k = self.filter_contour_candidates(self.storage[i])

            self.points_ordered.append(self.storage[self.k].get_xy())
            i = self.k
            if i == 0:
                break

    def get_contour(self):
        self.reorder()
        return self.points_ordered


if __name__ == '__main__':
    image_path = "../images/shape.jpg"
    det = ContourDetector(image_path)
    detected_contour = det.get_contour()
    print("detected contour: ", detected_contour)
    print(len(detected_contour))
