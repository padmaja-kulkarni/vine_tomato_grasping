"""
Created on Thu Feb 28 14:28:50 2013
Produces colormaps by curving through Lch color space, inspired by
"Lab color scale" by Steve Eddins 09 May 2006 (Updated 10 May 2006)
(BSD license)
http://www.mathworks.co.uk/matlabcentral/fileexchange/11037-lab-color-scale
Chroma should probably be limited to the RGB cube in a way that never
produces sharp angles in the curve, which appear as bands in the gradient.
Examples:
This is similar to the 'hsv' map, but isoluminant:
    lab_color_scale(hue=30, rot=1, l=75, chroma=41)
This is similar to 'cubehelix', except in Lch space instead of RGB space:
    lab_color_scale(hue=0, rot=-1.5, chroma=30)
"""
import numpy as np
from matplotlib import cm

"""
Both libraries produce similar, but not identical, results.
colormath fails more gracefully when exceeding the RGB color cube.
https://code.google.com/p/python-colormath/
http://markkness.net/colorpy/ColorPy.html
"""
from colormath.color_conversions import convert_color
from colormath.color_objects import LabColor, sRGBColor, HSVColor

def irgb_from_lab(l, a, b):
    rgb = np.array(convert_color(LabColor(l, a, b),
                              sRGBColor).get_value_tuple())
    return limit_rgb(rgb)

def irgb_from_hsv(h, s, v):
    rgb = np.array(convert_color(HSVColor(h, s, v),
                              sRGBColor).get_value_tuple())
    return limit_rgb(rgb)

def limit_rgb(rgb):
    return np.maximum(np.minimum(rgb, 1.0), 0.0)


def lab_color_scale(lutsize=256):
    """
    Color map created by drawing arcs through the L*c*h*
    (lightness, chroma, hue) cylindrical coordinate color space, also called
    the L*a*b* color space when using Cartesian coordinates.
    Parameters
    ----------
    lutsize : int
        The number of elements in the colormap lookup table. (Default is 256.)
    -------
    cmap : matplotlib.colors.LinearSegmentedColormap
        The resulting colormap object
    """

    a = np.linspace(-100, 100, lutsize)
    l = [50] * len(a)
    b = [0] * len(a)
    lab = np.vstack([l, a, b])

    rgbs = []

    for l, a, b in lab.T:
        r, g, b = irgb_from_lab(l, a, b)

        rgbs.append((r, g, b))

    return cm.colors.LinearSegmentedColormap.from_list('lab_color_scale', rgbs,
                                                       lutsize)

def hsv_color_scale(lutsize=256):
    """
    Color map created by drawing arcs through the L*c*h*
    (lightness, chroma, hue) cylindrical coordinate color space, also called
    the L*a*b* color space when using Cartesian coordinates.
    Parameters
    ----------
    lutsize : int
        The number of elements in the colormap lookup table. (Default is 256.)
    -------
    cmap : matplotlib.colors.LinearSegmentedColormap
        The resulting colormap object
    """

    h = np.linspace(0, 360, lutsize)
    s = [1.0] * len(h)
    v = [1.0] * len(h)
    hsv = np.vstack([h, s, v])

    rgbs = []

    for h, s, v in hsv.T:
        r, g, b = irgb_from_hsv(h, s, v)

        rgbs.append((r, g, b))

    return cm.colors.LinearSegmentedColormap.from_list('hsv_color_scale', rgbs,
                                                       lutsize)


if __name__ == "__main__":
    from matplotlib import pyplot as plt

    dx, dy = 0.01, 0.01

    x = np.arange(-2.0, 2.0001, dx)
    y = np.arange(-2.0, 2.0001, dy)
    X, Y = np.meshgrid(x, y)
    # Matplotlib's 2-bump example shows banding and other problems clearly
    Z = X * np.exp(-X**2 - Y**2)

    plt.figure()
    plt.imshow(Z, vmax=abs(Z).max(), vmin=-abs(Z).max(), cmap=hsv_color_scale())
    plt.colorbar()
    plt.show()