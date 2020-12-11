import rospy

def point_to_tomato(point, radius, frame):
    tomato = Tomato()
    tomato.header.frame_id = frame
    tomato.header.stamp = rospy.Time.now()

    tomato.position.x = point[0]
    tomato.position.y = point[1]
    tomato.position.z = point[2] + radius

    tomato.radius = radius
    return tomato


def generate_object(self):
    table_height = 0.23
    frame = "world"
    object_x = rospy.get_param("object_x")
    object_y = rospy.get_param("object_y")
    angle = rospy.get_param("object_angle")
    xyz = [object_x, object_y, 0.05 + table_height]
    rpy = [3.1415, 0, angle]  # 3.1415/2.0

    cage_pose = point_to_pose_stamped(xyz, rpy, frame, rospy.Time.now())

    L = 0.15
    peduncle = Peduncle()
    peduncle.pose = cage_pose
    peduncle.radius = 0.005
    peduncle.length = L

    radii = [0.05, 0.05]
    t1x = xyz[0] + (L / 2 + radii[0]) * np.cos(angle)
    t1y = xyz[1] - (L / 2 + radii[0]) * np.sin(angle)
    t2x = xyz[0] - (L / 2 + radii[1]) * np.cos(angle)
    t2y = xyz[1] + (L / 2 + radii[1]) * np.sin(angle)
    point1 = [t1x, t1y, table_height]
    point2 = [t2x, t2y, table_height]
    points = [point1, point2]

    tomatoes = []
    for point, radius in zip(points, radii):
        # tomatoes.append(point_to_tomato(point, radius, frame))
        pass

    truss = self.create_truss(tomatoes, cage_pose, peduncle)
    self.pub_object_features.publish(truss)
    return True


def create_truss(tomatoes, cage_pose, peduncle):

    truss = Truss()
    truss.tomatoes = tomatoes
    truss.cage_location = cage_pose
    truss.peduncle = peduncle

    return truss

def generate_peduncle(self, peduncle_features, cage_pose):

    peduncle = Peduncle()
    peduncle.pose = cage_pose
    # peduncle.pcl = peduncle_pcl
    peduncle.radius = 0.01
    peduncle.length = 0.15
    return peduncle


def segment_pcl(self, img, color=(255, 255, 255, 255)):

    r = color[0]
    g = color[1]
    b = color[2]
    a = color[3]

    index = np.nonzero(img)

    uvs = list()
    for row, col in zip(index[0], index[1]):
        uvs.append([col, row])

    rgba = struct.unpack('I', struct.pack('BBBB', b, g, r, a))[0]

    points = self.get_points(uvs=uvs, field_names=("x", "y", "z"))
    for i in range(0, len(points)):
        points[i] = points[i] + (rgba,)

    fields = [PointField('x', 0, PointField.FLOAT32, 1),
              PointField('y', 4, PointField.FLOAT32, 1),
              PointField('z', 8, PointField.FLOAT32, 1),
              PointField('rgba', 12, PointField.UINT32, 1)]

    return pc2.create_cloud(self.pcl.header, fields, points)


def visualize_tomatoes(self, tomato_mask):
    tomato_pcl = self.segment_pcl(tomato_mask, color=(200, 50, 50, 255))
    # self.pub_tomato_mask.publish(self.bridge.cv2_to_imgmsg(tomato_mask))
    self.pub_tomato_pcl.publish(tomato_pcl)

def visualize_peduncle(self, peduncle_mask):
    peduncle_pcl = self.segment_pcl(peduncle_mask, color=(50, 200, 50, 255))
    self.pub_peduncle_pcl.publish(peduncle_pcl)

def generate_tomatoes(self, tomato_features):

    tomatoes = []
    for i in range(0, len(tomato_features['col'])):
        # Load from struct
        col = tomato_features['col'][i]
        row = tomato_features['row'][i]
        radius = tomato_features['radii'][i]

        point = self.deproject(row, col)

        depth = self.get_depth(row, col)  # depth = self.depth_image[(row, col)]
        point1 = rs.rs2_deproject_pixel_to_point(self.intrin, [0, 0], depth)
        point2 = rs.rs2_deproject_pixel_to_point(self.intrin, [0, radius], depth)
        radius_m = euclidean(point1, point2)

        tomatoes.append(point_to_tomato(point, radius_m, self.camera_frame))

    return tomatoes