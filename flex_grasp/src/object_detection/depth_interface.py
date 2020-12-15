import numpy as np
import rospy


import sensor_msgs.point_cloud2 as pc2
import pyrealsense2 as rs

class DepthImageFilter(object):
    """ObjectDetection"""

    def __init__(self, image, intrinsics, patch_size=5, node_name="depth interface"):
        self.image = image
        self.intrinsics = intrinsics
        self.node_name = node_name
        self.patch_radius = int((patch_size - 1) / 2)

        # patch
        self.min_row = 0
        self.max_row = self.image.shape[0] - 1
        self.min_col = 0
        self.max_col = self.image.shape[1] - 1

    def generate_patch(self, row, col):
        """Returns a patch which can be used for filtering an image"""
        row_start = max([row - self.patch_radius, self.min_row])
        row_end = min([row + self.patch_radius, self.max_row])

        col_start = max([col - self.patch_radius, self.min_col])
        col_end = min([col + self.patch_radius, self.max_col])

        rows = np.arange(row_start, row_end + 1)
        cols = np.arange(col_start, col_end + 1)
        return rows, cols

    def get_point(self, row, col, depth=None, segment=None):
        """Get a 3D point a a certain (row, col) coordinate from a depth image"""
        # TODO: these should never be floats!
        row = int(row)
        col = int(col)

        if depth is None:
            depth = self.get_depth(row, col, segment=segment)

        if np.isnan(depth):
            rospy.logwarn("[OBJECT DETECTION] Computed depth is nan, can not compute point!")
            return 3 * [np.nan]

        # https://github.com/IntelRealSense/librealsense/wiki/Projection-in-RealSense-SDK-2.0
        pixel = [float(col), float(row)]  # [x, y]
        depth = float(depth)
        point = rs.rs2_deproject_pixel_to_point(self.intrinsics, pixel, depth)

        rospy.logdebug("[{0}] Point based on deprojection {1}".format(self.node_name, point))
        return point

    def get_depth(self, row, col, segment=None):
        """get the filtered depth from an depth image at location row, col"""
        # TODO: these should never be floats!
        row = int(row)
        col = int(col)

        rows, cols = self.generate_patch(row, col, self.image.shape)

        if segment is None:
            depth_patch = self.image[rows[:, np.newaxis], cols]
        else:
            depth_patch = self.image[segment > 0]

        non_zero = np.nonzero(depth_patch)
        depth_patch_non_zero = depth_patch[non_zero]
        return np.median(depth_patch_non_zero)


class PointCloudFilter(object):
    """ObjectDetection"""

    def __init__(self, pcl, patch_size=5, node_name="depth interface"):
        self.pcl = pcl
        self.node_name = node_name
        self.patch_radius = int((patch_size - 1) / 2)

        # patch
        self.min_row = 0
        self.max_row = self.pcl.height - 1
        self.min_col = 0
        self.max_col = self.pcl.width - 1

    def generate_patch(self, row, col):
        """Returns a patch which can be used for filtering a pointcloud"""

        row_start = max([row - self.patch_radius, self.min_row])
        row_end = min([row + self.patch_radius, self.max_row])

        col_start = max([col - self.patch_radius, self.min_col])
        col_end = min([col + self.patch_radius, self.max_col])

        rows = np.arange(row_start, row_end + 1)
        cols = np.arange(col_start, col_end + 1)

        uvs = list()  # [col, row]
        for col in cols:
            for row in rows:
                uvs.append([col, row])

        return uvs

    def get_point(self, row, col):
        """Get a 3D point at a certain (row,col) coordinate from a point cloud"""
        # TODO: these should never be floats!
        row = int(row)
        col = int(col)

        print '\n\n\n'
        print 'pcl.height: ', self.pcl.height
        print 'pcl.width: ', self.pcl.width

        uvs = self.generate_patch(row, col)
        point = self.get_point(self.pcl, uvs)
        rospy.logdebug("Point obtained from point cloud: %s", point)
        return point

    def get_depth(self, row, col, depth_image, segment=None):
        """get the filtered depth from an depth image at location row, col"""
        # TODO: these should never be floats!
        row = int(row)
        col = int(col)

        rows, cols = self.generate_patch(row, col, depth_image.shape)

        if segment is None:
            depth_patch = depth_image[rows[:, np.newaxis], cols]
        else:
            depth_patch = depth_image[segment > 0]

        non_zero = np.nonzero(depth_patch)
        depth_patch_non_zero = depth_patch[non_zero]
        return np.median(depth_patch_non_zero)

    def get_point(self, uvs):
        points = self.get_points(uvs=uvs)
        point = np.mean(points, axis=0)
        return point

    def get_points(self, uvs=[], field_names=("x", "y", "z")):
        points = list(pc2.read_points(self.pcl, skip_nans=False, field_names=field_names, uvs=uvs))
        return points