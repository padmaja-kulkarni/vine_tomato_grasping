import numpy as np


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
        if self.frame_id == frame_id:
            return self.coord
        else:
            coord = self.transform.apply(self, frame_id)
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

        else: # (self.frame_id == point.frame_id) or (transform is not None):
            coord = point.get_coord(self.frame_id)
            return np.sqrt(np.sum(np.power(np.subtract(self.coord, coord), 2)))

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


class Transform(object):
    """
    Very simple class used for storing a two-dimensional transformation, and applying it to two-dimensional points
    Note that it does not support multiple transformations or and successive transformations.
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
    """Takes a list of points, and outputs a list of coordinates"""
    coords = []
    for point in point_list:
        coords.append(point.get_coord(frame))

    return coords


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
