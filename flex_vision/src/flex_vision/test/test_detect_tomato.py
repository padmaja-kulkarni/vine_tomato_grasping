from matplotlib import pyplot as plt
from flex_vision.utils import util
import numpy as np
from flex_vision.detect_truss.ProcessImage import ProcessImage

shape = [16, 9]
dpi = 100

plt.figure(figsize=shape, dpi=100)
plt.tight_layout()
ax = plt.gca()
util.clear_axis()
plt.xlim(0, 1920)
plt.ylim(0, 1080)

centers = [[850, 450], [650, 250]]
radii = [150, 120]

tomatoes = {'centers': centers,
            'radii': radii}

peduncle = {'centers': [[750, 350], [780-50, 320-50], [720+50, 380+50]], 'angles': [np.deg2rad(-45), np.deg2rad(45), np.deg2rad(45)], 'length': [300, 100, 100]}

util.add_rectangle([0, 0], width=1920, height=1080, fc=util.background_color, linewidth=0)
util.plot_truss(tomato=tomatoes, peduncle=peduncle)

name = 'temp.png'
plt.savefig(name)
img = util.load_rgb(name, horizontal=True)
plt.imshow(img)
plt.show()

px_per_mm = 3.75

process_image = ProcessImage(use_truss=True)
process_image.add_image(img, px_per_mm=px_per_mm, name='test')

success = process_image.process_image()
process_image.get_truss_visualization(local=True, save=False)
plt.show()


features = process_image.get_object_features()
print features