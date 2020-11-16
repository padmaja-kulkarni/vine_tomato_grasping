#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import numpy as np


def from_coord_list(coords, frame):
    "Takes a list of coordinates, and outputs a list of Point2D"
    if coords is None:
        return None

    point_list = []
    for coord in coords:
        point = Point2D(coord, frame)
        point_list.append(point)

    return point_list

def get_coord_list(point_list, transform, frame):
    "Takes a list of points, and outputs a list of coordinates"
    coords = []
    for point in point_list:
        coords.append(point.get_coord(transform, frame))

    return coords  # np.array(coords, ndmin=2)

class Point2D(object):
    """
    class used for storing two-dimensional points, and getting the coordiante of a point with respect to a certain
    reference frame
    """

    def __init__(self, coord, frame_id, transform=None):
        """
        coord: two-dimensional coordinates as [x, y]
        frame_id: the name of the frame
        """
        self.coord = coord
        self.frame_id = frame_id
        self.transform = transform

    def get_coord(self, transform, frame_id):
        """
        Get the coordinate of a two-dimensional point, with respect to a certain frame
        """
        if self.frame_id != frame_id:
            coord = transform.apply(self, frame_id)
        else:
            coord = self._coord
        return [coord[0, 0], coord[1, 0]]  # np.array(

    def dist(self, points, transform=None):
        """
        Calculate the distance between two points
        """
        if isinstance(points, (list, tuple)):
            distances = []

            for point in points:
                distances.append(self._dist(point, transform=transform))

            return distances

        else:
            return self._dist(points, transform=transform)


    def _dist(self, point, transform=None):

        if (self.frame_id != point.frame_id) and (transform is None):
            print "points are defined with respect to different frames, please provide a transform"
            return None

        if (self.frame_id != point.frame_id) and (transform is not None):
            coord = point.get_coord(transform, self.frame_id)
            return np.sqrt(np.sum(np.power(self._coord - coord, 2)))

        return np.sqrt(np.sum(np.power(self._coord - point.coord, 2)))

    @property
    def coord(self):
        return self._coord.tolist()

    @coord.setter
    def coord(self, coord):
        if isinstance(coord, (list, tuple)):
            if len(coord) == 2:
                self._coord = np.array(coord, ndmin=2).transpose()
            else:
                print "please use 2d coordinates"

        elif isinstance(coord, np.ndarray):
            coord = np.array(coord, ndmin=2)
            if coord.shape == (1, 2):
                self._coord = coord.transpose()
            elif coord.shape == (2, 1):
                self._coord = coord
            else:
                print "please use 2d coordinates"


def main():

    frame_id = 'original'
    coord = [0, 0]
    point1 = Point2D(coord, frame_id)

    frame_id = 'original'
    coord = [1, 1]
    point2 = Point2D(coord, frame_id)

    print point1.dist(point2)

if __name__ == '__main__':
    main()