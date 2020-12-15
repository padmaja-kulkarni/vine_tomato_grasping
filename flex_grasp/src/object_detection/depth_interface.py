import numpy as np
import rospy


import sensor_msgs.point_cloud2 as pc2
import pyrealsense2 as rs

# numpy array printed up to mm
np.set_printoptions(precision=3)

class DepthImageFilter(object):
    """DepthImageFilter"""

    def __init__(self, image, intrinsics, patch_size=5, node_name="depth interface"):
        self.image = image
        self.intrinsics = intrinsics
        self.node_name = node_name
        self.patch_radius = int((patch_size - 1) / 2)

        # patch
        self.min_row = 0
        self.max_row = self.image.shape[0]
        self.min_col = 0
        self.max_col = self.image.shape[1]

    def generate_patch(self, row, col):
        """Returns a patch which can be used for filtering an image"""
        row = int(round(row))
        col = int(round(col))

        row_start = max([row - self.patch_radius, self.min_row])
        row_end = min([row + self.patch_radius, self.max_row - 1]) + 1

        col_start = max([col - self.patch_radius, self.min_col])
        col_end = min([col + self.patch_radius, self.max_col - 1]) + 1

        rows = np.arange(row_start, row_end)
        cols = np.arange(col_start, col_end)
        return rows, cols

    def deproject(self, row, col, depth=None, segment=None):
        """
        Deproject a 2D coordinate to a 3D point using the depth image, if an depth is provided the depth image is
        ignored.
        """

        if depth is None:
            depth = self.get_depth(row, col, segment=segment)

        if np.isnan(depth):
            rospy.logwarn("[OBJECT DETECTION] Computed depth is nan, can not compute point!")
            return 3 * [np.nan]

        point = rs.rs2_deproject_pixel_to_point(self.intrinsics, [col, row], depth)
        rospy.logdebug("[{0}] Point obtained from depth image {1}".format(self.node_name, np.array(point)))
        return point

    def get_depth(self, row, col, segment=None):
        """get the filtered depth from an depth image at location row, col"""
        rows, cols = self.generate_patch(row, col)

        if segment is None:
            depth_patch = self.image[rows[:, np.newaxis], cols]
        else:
            depth_patch = self.image[segment > 0]

        non_zero = np.nonzero(depth_patch)
        depth_patch_non_zero = depth_patch[non_zero]
        return np.median(depth_patch_non_zero)


class PointCloudFilter(object):
    """PointCloudFilter"""

    def __init__(self, pcl, patch_size=5, node_name="depth interface"):
        self.pcl = pcl
        self.node_name = node_name
        self.patch_radius = int((patch_size - 1) / 2)

        # patch
        self.min_row = 0
        self.max_row = self.pcl.height
        self.min_col = 0
        self.max_col = self.pcl.width

    def generate_patch(self, row, col):
        """Returns a patch which can be used for filtering a pointcloud"""
        row = int(round(row))
        col = int(round(col))

        row_start = max([row - self.patch_radius, self.min_row])
        row_end = min([row + self.patch_radius, self.max_row - 1]) + 1

        col_start = max([col - self.patch_radius, self.min_col])
        col_end = min([col + self.patch_radius, self.max_col - 1]) + 1

        rows = np.arange(row_start, row_end)
        cols = np.arange(col_start, col_end)

        uvs = list()
        for col in cols:
            for row in rows:
                uvs.append([col, row])

        return uvs

    def deproject(self, row, col):
        """Deproject a 2D coordinate to a 3D point using the point cloud"""
        uvs = self.generate_patch(row, col)
        point = self.get_point(uvs)
        rospy.logdebug("[{0}] Point obtained from point cloud {1}".format(self.node_name, point))
        return point

    def get_depth(self, row, col, segment=None):
        """get the filtered depth from an depth image at location row, col"""
        uvs = self.generate_patch(row, col)
        point = self.get_point(uvs)
        return point[2]

    def get_point(self, uvs):
        """returns the mean of the points at coordinates uvs"""
        points = self.get_points(uvs=uvs)
        print np.array(points)

        # TODO: use median instead of mean
        point = np.mean(points, axis=0)
        return point

    def get_points(self, uvs=[], field_names=("x", "y", "z")):
        """Reads points from the point cloud with fields called fieldnames at coordinates uvs"""
        points = list(pc2.read_points(self.pcl, skip_nans=False, field_names=field_names, uvs=uvs))
        return points
