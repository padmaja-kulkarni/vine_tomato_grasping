import numpy as np


class Transform(object):
    """
    class used for storing two-dimensional transforms, and applying these to two-dimensional points
    """

    def __init__(self, from_frame_id, to_frame_id, dim=None, angle=None, translation=None):
        """
        from_frame_id: transform from frame name
        to_frame_id: transform to frame name
        dim: image dimensions [height, width]
        angle: angle of rotation in radians
        translation: translation
        """
        if (dim is not None) and (angle is not None):
            height = dim[0]
            width = dim[1]

            R = np.array([[np.cos(angle), np.sin(angle)], [-np.sin(angle), np.cos(angle)]])

            if angle > 0:
                T = np.array([[-np.sin(angle) * width], [0]])
            else:
                T = np.array([[0], [np.sin(angle) * height]])

        elif angle is None:
            R = np.identity(2)
            T = np.zeros((2, 1))

        elif dim is None:
            print "Did not specify image dimensions, ignoring rotation!"

        self.from_frame_id = from_frame_id
        self.to_frame_id = to_frame_id
        self.R = R
        self.Rinv = np.linalg.inv(self.R)
        self.T = T
        if translation is not None:
            self.translation = ensure_vector(translation)
        else:
            self.translation = np.zeros((2, 1))

    def apply(self, point, to_frame_id):
        """
        Applies transfor to a given point to a given frame
        point: Point object
        to_frame_id: string, name of frame id
        """
        if point.frame_id == to_frame_id:
            return point._coord
        if point.frame_id == self.from_frame_id and to_frame_id == self.to_frame_id:
            return self._forwards(point._coord)
        elif point.frame_id == self.to_frame_id and to_frame_id == self.from_frame_id:
            return self._backwards(point._coord)
        else:
            print('Unknown frame id')
            return None

    def _forwards(self, coord):
        """
        translates 2d coordinate with and angle and than translation
        coord: 2D coords [x, y]
        """
        coord = np.array([[coord[1, 0], coord[0, 0]]]).T  # [x, y] --> [r, c]
        coord = np.matmul(self.Rinv, coord) - self.T - self.translation
        return np.array([[coord[1, 0], coord[0, 0]]]).T  # [r, c] --> [x, y]

    def _backwards(self, coord):
        """
        translates 2d coordiante with -translation and than -angle
        coord: 2D coords [x, y]
        """
        coord = np.array([[coord[1, 0], coord[0, 0]]]).T  # [x, y] --> [r, c]
        coord = np.matmul(self.R, coord + self.T + self.translation)
        return np.array([[coord[1, 0], coord[0, 0]]]).T  # [r, c] --> [x, y]


def ensure_vector(data):
    if isinstance(data, (list, tuple)):
        if len(data) == 2:
            return np.array(data, ndmin=2).transpose()
        else:
            print "please use 2d coordinates"
            return None

    elif isinstance(data, np.ndarray):
        coord = np.array(data, ndmin=2)
        if coord.shape == (1, 2):
            return coord.transpose()
        elif coord.shape == (2, 1):
            return coord
        else:
            print "please use 2d coordinates"
            return None