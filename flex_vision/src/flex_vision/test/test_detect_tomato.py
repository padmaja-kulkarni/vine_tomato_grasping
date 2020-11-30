import unittest
from matplotlib import pyplot as plt
from flex_vision.utils import util
import numpy as np
from flex_vision.detect_truss.ProcessImage import ProcessImage
from flex_vision.analyze_results.analyze_results import index_true_positives


class DetectTrussTests(unittest.TestCase):
    visualize = True

    def test_detect_truss(self):
        process_image = ProcessImage()
        radius_min_mm = process_image.settings['detect_tomato']['radius_min_mm']
        radius_max_mm = process_image.settings['detect_tomato']['radius_max_mm']

        # generate truss
        px_per_mm = 3.75
        radii_range = [radius_min_mm * px_per_mm, radius_max_mm * px_per_mm]
        tomatoes, peduncle = generate_truss_features([750, 350], radii_range)

        # generate background image
        bg_img = np.tile(np.array(util.background_color, ndmin=3, dtype=np.uint8), (1080, 1920, 1))

        # plot truss
        plt.figure(figsize=[16, 9], dpi=120)
        util.plot_image(bg_img)
        util.plot_truss(tomato=tomatoes, peduncle=peduncle)
        util.clear_axis()
        img = util.figure_to_image(plt.gcf())

        # process the generated image
        process_image.add_image(img, px_per_mm=px_per_mm, name='test')
        process_image.process_image()
        features_prediction = process_image.get_object_features()

        if self.visualize:
            plt.imshow(img)
            plt.show()

            process_image.get_truss_visualization(local=False, save=False)
            util.clear_axis()
            plt.show()

        # analyze results
        i_prediction, i_label, false_pos, false_neg = index_true_positives(tomatoes['centers'],
                                                                           features_prediction['tomato']['centers'], 10)

        centers_prediction = np.array(features_prediction['tomato']['centers'])[i_prediction]
        radii_prediction = np.array(features_prediction['tomato']['radii'])[i_prediction]

        centers_label = np.array(tomatoes['centers'])[i_label]
        radii_label = np.array(tomatoes['radii'])[i_label]


        print 'predicted features, centers: ', centers_prediction, ' radii: ', radii_prediction
        print 'actual features, centers: ', centers_label, ' radii: ', radii_label
        np.testing.assert_almost_equal(0, 0)

def generate_truss_features(truss_center, radii_range, angle_deg=30):
    """
        center: truss center placement
        angle: truss angle (in degree)
    """
    x = truss_center[0]
    y = truss_center[1]
    radius_min = radii_range[0]
    radius_max = radii_range[1]
    tomato_dist = 120

    angle_rad = np.deg2rad(angle_deg)
    rotation_matrix = np.array(((np.cos(angle_rad), -np.sin(angle_rad)), (np.sin(angle_rad), np.cos(angle_rad))))
    shift = 30

    # define truss features
    tomatoes = {'centers': [rotate_point([shift, -tomato_dist], rotation_matrix, truss_center),
                            rotate_point([-shift, tomato_dist], rotation_matrix, truss_center)],  # [x, y]
                'radii': [radius_min, radius_max]}

    peduncle = {'centers': [[x, y],
                            rotate_point([shift, -tomato_dist/2], rotation_matrix, truss_center),
                            rotate_point([-shift, tomato_dist/2], rotation_matrix, truss_center)],
                'angles': [-angle_rad, np.pi/2 - angle_rad, np.pi/2- angle_rad],
                'length': [300, tomato_dist, tomato_dist],
                'junctions': [rotate_point([shift, 0], rotation_matrix, truss_center),
                              rotate_point([-shift, 0], rotation_matrix, truss_center)]}

    return tomatoes, peduncle

def rotate_point(point, rot_mat, transform):
    vec = np.matmul(np.array(point, ndmin=2), rot_mat) + np.array(transform, ndmin=2)
    return vec[0].tolist()

if __name__ == '__main__':
    unittest.main()