import numpy as np
import skan

n = 10
img = np.zeros([10,10], dtype=np.uint8)
img[6, :] = 255
img[:, 6] = 255

print(img)

graph, pixel_coordinates, degree_image = skan.skeleton_to_csgraph(img, unique_junctions=True)

skeleton = skan.Skeleton(img, unique_junctions=True)
branch_data = skan.summarize(skeleton)

src_node_id = np.unique(branch_data['node-id-src'].values)
dst_node_id = np.unique(branch_data['node-id-dst'].values)
all_node_id = np.unique(np.append(src_node_id, dst_node_id))

end_node_index = skeleton.degrees[all_node_id] == 1

end_node_id = all_node_id[end_node_index]
junc_node_id = np.setdiff1d(all_node_id, end_node_id)

end_node_coord = skeleton.coordinates[end_node_id][:, [1, 0]]
junc_node_coord = skeleton.coordinates[junc_node_id][:, [1, 0]]


print(img)
print(degree_image)
print(skeleton.degrees_image)