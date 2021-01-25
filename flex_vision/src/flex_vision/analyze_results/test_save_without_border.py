import numpy as np
import matplotlib.pyplot as plt

def save_image(data, filename):
    sizes = np.shape(data)
    fig = plt.figure()
    fig.set_size_inches(float(sizes[1])/float(sizes[0]), 1, forward=False)
    ax = plt.Axes(fig, [0., 0., 1., 1.])
    ax.set_axis_off()
    fig.add_axes(ax)
    ax.imshow(data)

    # circle1 = plt.Circle((100, 100), 50, color='r', zorder=10)
    # ax.add_patch(circle1)

    plt.subplots_adjust(top=1, bottom=0, right=1, left=0, hspace=0, wspace=0)
    plt.margins(0, 0)
    plt.gca().xaxis.set_major_locator(plt.NullLocator())
    plt.gca().yaxis.set_major_locator(plt.NullLocator())

    plt.savefig(filename, dpi=sizes[0], cmap='hot') #  , bbox_inches='tight', pad_inches=0)
    plt.close()

def main():
    data = np.random.randint(0, 100, (256, 256*2))
    save_image(data, '1.png')
    save_image(data, '1.pdf')


if __name__ == '__main__':
    main()
