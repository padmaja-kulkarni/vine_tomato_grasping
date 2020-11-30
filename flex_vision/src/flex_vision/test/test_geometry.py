import unittest
import numpy as np
from flex_vision.utils.geometry import Point2D, Transform, MissingTransformError, LengthMismatchError


class TransformTests(unittest.TestCase):

    def test_translation(self):
        transform = Transform('origin', 'local', translation=[3, 4])

        point1 = Point2D([0, 0], 'origin', transform)
        point2 = Point2D([0, 0], 'local', transform)

        np.testing.assert_almost_equal(point1.get_coord('origin'), [0, 0])
        np.testing.assert_almost_equal(point1.get_coord('local'), [-3, -4])

        np.testing.assert_almost_equal(point2.get_coord('origin'), [3, 4])
        np.testing.assert_almost_equal(point2.get_coord('local'), [0, 0])

        np.testing.assert_almost_equal(point1.dist(point2), 5)

    def test_90deg_rotation(self):

        shape = [20, 10]  # [width, height]
        angle = np.pi/2
        transform = Transform('origin', 'local', dim=shape, angle=angle)

        point1 = Point2D([10, 5], 'origin', transform)
        point2 = Point2D([5, 10], 'local', transform)

        np.testing.assert_almost_equal(point1.get_coord('origin'), [10, 5])
        np.testing.assert_almost_equal(point1.get_coord('local'), [5, 10])

        np.testing.assert_almost_equal(point2.get_coord('origin'), [10, 5])
        np.testing.assert_almost_equal(point2.get_coord('local'), [5, 10])

        np.testing.assert_almost_equal(point1.dist(point2), 0)

    def test_45deg_rotation(self):

        shape = [20, 20]  # [width, height]
        angle = np.pi/4
        transform = Transform('origin', 'local', dim=shape, angle=angle)

        point1 = Point2D([10, 10], 'origin', transform)
        point2 = Point2D([np.sqrt(2)*10, np.sqrt(2)*10], 'local', transform)

        np.testing.assert_almost_equal(point1.get_coord('origin'), [10, 10])
        np.testing.assert_almost_equal(point1.get_coord('local'), [np.sqrt(2)*10, np.sqrt(2)*10])

        np.testing.assert_almost_equal(point2.get_coord('origin'), [10, 10])
        np.testing.assert_almost_equal(point2.get_coord('local'), [np.sqrt(2)*10, np.sqrt(2)*10])

        np.testing.assert_almost_equal(point1.dist(point2), 0)

    def test_345_rotation(self):

        shape = [8, 6]  # [width, height]
        angle = -np.arctan2(3, 4)
        transform = Transform('origin', 'local', dim=shape, angle=angle)

        point1 = Point2D([4, 3], 'origin', transform)
        point2 = Point2D([5, 3.0/5.0*8], 'local', transform)

        np.testing.assert_almost_equal(point1.get_coord('origin'), [4, 3])
        np.testing.assert_almost_equal(point1.get_coord('local'), [5, 3.0/5.0*8])

        np.testing.assert_almost_equal(point2.get_coord('origin'), [4, 3])
        np.testing.assert_almost_equal(point2.get_coord('local'), [5, 3.0/5.0*8])

        np.testing.assert_almost_equal(point1.dist(point2), 0)

    def test_transform_forwards_backwards(self):
        """
        get coordinate return the original coordinate after translating forwards and backwards
        """
        shape = [1000, 400]  # [width, height]
        translation = [50, -10]
        for angle_rad in np.arange(-np.pi, np.pi, 10):

            transform = Transform('origin', 'local', shape, angle_rad, translation)
            point1 = Point2D([400, 100], 'origin', transform)

            coord1 = point1.get_coord('local')
            point2 = Point2D(coord1, 'local', transform)

            np.testing.assert_almost_equal(point1.get_coord('origin'), point2.get_coord('origin'))


    def test_missing_transform(self):
        """
        get coordinate returns a MissingTransformError when requested a coordinate when no transform was given
        """
        point1 = Point2D([400, 100], 'origin')
        self.assertRaises(MissingTransformError, point1.get_coord, 'local')

    def test_missing_transform(self):
        """
        get coordinate returns a MissingTransformError when requested a coordinate in a frame for which the transform is unknown
        """
        transform = Transform('origin', 'local', translation=[0, 0])
        point1 = Point2D([400, 100], 'origin', transform)
        self.assertRaises(MissingTransformError, point1.get_coord, 'space')

    def test_point_length_mismatch(self):
        """
        Point2D returns a LengthMismatchError when a wrong coordinate length is provided
        """
        self.assertRaises(LengthMismatchError, Point2D, [400, 100, 100], 'origin')

    def test_transform_length_mismatch(self):
        """
        Transform returns a LengthMismatchError when a wrong translation length is provided
        """
        self.assertRaises(LengthMismatchError, Transform, 'origin', 'local', translation=[0, 0, 0])


if __name__ == '__main__':
    unittest.main()