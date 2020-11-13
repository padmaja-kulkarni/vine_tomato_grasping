import numpy as np


class Transform(object):

    def __init__(self, from_frame_id, to_frame_id, dim, angle, translation=None):
        """
        angle: in radians
        translation:
        dim: image dimensions [height, width]
        """

        height = dim[0]
        width = dim[1]

        R = np.array([[np.cos(angle), np.sin(angle)], [-np.sin(angle), np.cos(angle)]])

        if angle > 0:
            T = np.array([[-np.sin(angle) * width], [0]])
        else:
            T = np.array([[0], [np.sin(angle) * height]])

        self.from_frame_id = from_frame_id
        self.to_frame_id = to_frame_id
        self.R = R
        self.Rinv = np.linalg.inv(self.R)
        self.T = T
        if translation is not None:
            self.translation = translation
        else:
            self.translation = np.zeros((2, 1))

    def apply(self, point, to_frame_id):
        """
        Applies transfor to a given point to a given frame
        point: Point object
        to_frame_id: string, name of frame id
        """
        if point.frame_id == to_frame_id:
            return point.coord
        if point.frame_id == self.from_frame_id and to_frame_id == self.to_frame_id:
            return self.forwards(point.coord)
        elif point.frame_id == self.to_frame_id and to_frame_id == self.from_frame_id:
            return self.backwards(point.coord)
        else:
            print('Unknown frame id')
            return None

    def forwards(self, coord):
        """
        translates 2d coordinate with and angle and than translation
        coord: 2D coords [x, y]
        """
        coord = np.array([[coord[1, 0], coord[0, 0]]]).T
        coord = np.matmul(self.Rinv, coord) - self.T - self.translation
        return np.array([[coord[1, 0], coord[0, 0]]]).T

    def backwards(self, coord):
        """
        translates 2d coordiante with -translation and than -angle
        coord: 2D coords [x, y]
        """
        coord = np.array([[coord[1, 0], coord[0, 0]]]).T
        coord = np.matmul(self.R, (coord + self.T + self.translation))
        return np.array([[coord[1, 0], coord[0, 0]]]).T

class Point2D(object):

    def __init__(self, coord, frame_id):

        self.coord = coord
        self.frame_id = frame_id

    def get_coord(self, transform, to_frame_id):
        coord = transform.apply(self, to_frame_id)
        return np.array([coord[0, 0], coord[1, 0]])

    @property
    def coord(self):
        return self.__coord  #  np.array([self.__coord[0, 0], self.__coord[1, 0]])

    @coord.setter
    def coord(self, coord):
        if isinstance(coord, (list, tuple)):
            if len(coord) == 2:
                self.__coord = np.array(coord, ndmin=2).transpose()
            else:
                print "please use 2d coordinates"

        elif isinstance(coord, np.ndarray):
            coord = np.array(coord, ndmin=2)
            if coord.shape == (1, 2):
                self.__coord = coord.transpose()
            elif coord.shape == (2, 1):
                self.__coord = coord
            else:
                print "please use 2d coordinates"