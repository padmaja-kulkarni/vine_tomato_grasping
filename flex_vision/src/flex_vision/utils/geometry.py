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
        elif self.transform is None:
            raise MissingTransformError(self.transform, from_frame=self.frame_id, to_frame=frame_id)
        else:
            coord = self.transform.apply(self, frame_id)
            return coord[:, 0].tolist()

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
        # TODO: we can also apply the transform stored in point(s)

        transform = self.transform

        if (self.frame_id != point.frame_id) and (transform is None):
            raise MissingTransformError(transform, from_frame=self.frame_id, to_frame=point.frame_id)

        else:  # (self.frame_id == point.frame_id) or (transform is not None):
            coord = point.get_coord(self.frame_id)
            return np.sqrt(np.sum(np.power(np.subtract(self.coord, coord), 2)))

    @property
    def coord(self):
        return self._coord[:, 0].tolist()

    @coord.setter
    def coord(self, coord):
        self._coord = vectorize(coord)


class Transform(object):
    """
    Very simple class used for storing a two-dimensional transformation, and applying it to two-dimensional points
    Note that it does not support multiple transformations or and successive transformations.
    """

    # TODO: move from row, column coordinates to xy, to make things less confusing.

    def __init__(self, from_frame_id, to_frame_id, dim=None, angle=None, translation=None):
        """
        from_frame_id: transform from frame name
        to_frame_id: transform to frame name
        dim: image dimensions [width, height]
        angle: angle of rotation in radians (counter clockwise is positive)
        translation: translation [x,y]
        """
        if (dim is not None) and (angle is not None):
            height = dim[1]
            width = dim[0]

            # Rotation matrix
            R = np.array([[np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]])

            # Translation vector due to rotation
            if angle > 0:
                T1 = [0, -np.sin(angle) * width]
            else:
                T1 = [np.sin(angle) * height, 0]

            # Translation vector due to >90deg rotations
            if (angle < -np.pi/2) or (angle > np.pi/2):
                T2 = [np.cos(angle) * width, np.cos(angle) * height]
            else:
                T2 = [0, 0]

            T = vectorize(T1) + vectorize(T2)

        else:
            R = np.identity(2)
            T = np.zeros((2, 1))

            if angle is not None:
                print "Did not specify image dimensions, ignoring rotation!"

        self.from_frame_id = from_frame_id
        self.to_frame_id = to_frame_id
        self.R = R
        self.Rinv = np.linalg.inv(self.R)
        self.T = T
        if translation is not None:
            self.translation = vectorize(translation)
        else:
            self.translation = np.zeros((2, 1))

    def apply(self, point, to_frame_id):
        """
        Applies transform to a given point to a given frame
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
            raise MissingTransformError(self, from_frame=point.frame_id, to_frame=to_frame_id)

    def _forwards(self, coord):
        """
        translates 2d coordinate with and angle and than translation
        coord: 2D coords [x, y]
        """
        return np.matmul(self.Rinv, coord) - self.T - self.translation

    def _backwards(self, coord):
        """
        translates 2d coordiante with -translation and than -angle
        coord: 2D coords [x, y]
        """
        return np.matmul(self.R, coord + self.T + self.translation)


class MissingTransformError(Exception):
    """Exception raised for errors in the input."""
    def __init__(self, transform=None, from_frame=None, to_frame=None):
        self.transform = transform
        self.from_frame = from_frame
        self.to_frame = to_frame

    def __str__(self):
        if (self.transform is None) and (self.from_frame is not None) and (self.to_frame is not None) :
            return "Cannot transform from " + self.from_frame + " frame to " + self.to_frame + " frame, transform is empty!"
        elif self.transform is None:
            return "Cannot transform, transform is empty!"
        else:
            return "Cannot transform from " + self.from_frame + " frame to " + self.to_frame + " frame, transform unknown!"


class LengthMismatchError(Exception):
    """Exception raised for errors in the input."""
    def __init__(self, given_length=None, desired_length=None):
        self.given_length = given_length
        self.desired_length = desired_length

    def __str__(self):
        if (self.given_length is not None) and (self.desired_length is not None):
            return "Provided wrong length: you gave " + str(self.given_length) + ", but should be " + str(self.desired_length) + "!"
        elif self.desired_length is not None:
            return "Provided wrong length: please provide an input of length" + str(self.desired_length) + "!"
        else:
            return "Provided wrong length!"


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


def vectorize(data):
    """Takes a list, tuple or numpy array and returns a column vector"""
    if isinstance(data, (list, tuple)):
        if len(data) == 2:
            return np.array(data, ndmin=2).transpose()
        else:
            raise LengthMismatchError(given_length=len(data), desired_length=2)

    elif isinstance(data, np.ndarray):
        coord = np.array(data, ndmin=2)
        if coord.shape == (1, 2):
            return coord.transpose()
        elif coord.shape == (2, 1):
            return coord
        else:
            raise LengthMismatchError(desired_length=2)


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
