#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import numpy as np
from transform import Transform

def points_from_coords(coords, frame, transform=None):
    "Takes a list of coordinates, and outputs a list of Point2D"
    if coords is None:
        return None

    point_list = []
    for coord in coords:
        point = Point2D(coord, frame, transform)
        point_list.append(point)

    return point_list

def coords_from_points(point_list, frame):
    "Takes a list of points, and outputs a list of coordinates"
    coords = []
    for point in point_list:
        coords.append(point.get_coord(frame))

    return coords

class Point2D(object):
    """
    class used for storing two-dimensional points, and getting the coordiante of a point with respect to a certain
    reference frame
    """

    # TODO: currently only a single transform is supported.

    def __init__(self, coord, frame_id, transform=None):
        """
        coord: two-dimensional coordinates as [x, y]
        frame_id: the name of the frame
        """
        self.coord = coord
        self.frame_id = frame_id
        self.transform = transform

    def get_coord(self, frame_id):
        """
        Get the coordinate of a two-dimensional point, with respect to a certain frame
        """
        if self.frame_id != frame_id:
            coord = self.transform.apply(self, frame_id)
        else:
            coord = self._coord
        return [coord[0, 0], coord[1, 0]]

    def dist(self, points):
        """
        Calculate the distance between two points
        """

        if isinstance(points, (list, tuple)):
            distances = []

            for point in points:
                distances.append(self._dist(point))

            return distances

        else:
            return self._dist(points)

    def _dist(self, point):
        # TODO: we can also you the transform stored in point(s)

        transform = self.transform

        if (self.frame_id != point.frame_id) and (transform is None):
            print "points are defined with respect to different frames, please provide a transform"
            return None

        elif (self.frame_id != point.frame_id) and (transform is not None):
            coord = point.get_coord(self.frame_id)
            return np.sqrt(np.sum(np.power(np.subtract(self.coord, coord), 2)))

        else:
            return np.sqrt(np.sum(np.power(np.subtract(self.coord, point.coord), 2)))

    @property
    def coord(self):
        return self._coord[:, 0].tolist()

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
    """
    Brief demo of the Point2d and Transform class
    """
    transform = Transform('origin', 'local', translation=(3, 4))

    frame_id = 'origin'
    coord = [0, 0]
    point1 = Point2D(coord, frame_id, transform)

    frame_id = 'local'
    coord = [0, 0]
    point2 = Point2D(coord, frame_id, transform)

    for frame_id in ['origin', 'local']:
        print 'point 1 in ', frame_id, ': ', point1.get_coord(frame_id)
        print 'point 2 in ', frame_id, ': ', point2.get_coord(frame_id)

    print 'distance between points: ', point1.dist(point2)


if __name__ == '__main__':
    main()