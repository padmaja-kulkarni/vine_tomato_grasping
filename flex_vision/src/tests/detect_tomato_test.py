from detect_truss import detect_tomato
from matplotlib import pyplot as plt
from utils import util
import numpy as np


shape = [16, 9]
dpi = 100

plt.figure(figsize=shape, dpi =100)
plt.tight_layout()
ax = plt.gca()
util.clear_axis()
plt.xlim(0, 1920)
plt.ylim(0, 1080)

centers = [[200, 800], [300, 700]]
radii = [100, 100]

tomatoes = {'centers': centers,
            'radii': radii}

util.add_rectangle([0,0], width=1920, height=1080, fc=util.background_color, linewidth=0)
util.plot_truss(tomato=tomatoes)

name = 'temp.png'
plt.savefig(name)
img = util.load_rgb(name, horizontal=True)
plt.imshow(img)
plt.show()