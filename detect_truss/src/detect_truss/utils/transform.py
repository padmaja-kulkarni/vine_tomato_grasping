import numpy as np
import warnings

def rot2or(loc, dim, alpha):
    # loc in [rows, cols]

    N = loc.shape[0]
    LOC = np.empty((N, 2))

    for i in range(0, N):
        col = loc[i, 0]
        row = loc[i, 1]
        H = dim[0]
        W = dim[1]

        if (alpha > np.pi or alpha < -np.pi):
            warnings.warn('Are you using radians?')

        # trig equations depend on angle
        if alpha < 0:
            COL = col * np.cos(alpha) - row * np.sin(alpha) + np.cos(alpha) * np.sin(alpha) * H
            ROW = col * np.sin(alpha) + row * np.cos(alpha) + np.sin(alpha) * np.sin(alpha) * H
        else:
            COL = col * np.cos(alpha) - row * np.sin(alpha) + np.sin(alpha) * np.sin(alpha) * W
            ROW = col * np.sin(alpha) + row * np.cos(alpha) - np.cos(alpha) * np.sin(alpha) * W

        LOC[i, :] = np.matrix((COL, ROW))
    return LOC


def translation_rot2or(dim, alpha):
    H = dim[0]
    W = dim[1]

    if (alpha > np.pi or alpha < -np.pi):
        warnings.warn('Are you using radians?')

    # trig equations depend on angle
    if alpha < 0:
        col = np.cos(alpha) * np.sin(alpha) * H
        row = np.sin(alpha) * np.sin(alpha) * H
    else:
        col = np.sin(alpha) * np.sin(alpha) * W
        row = np.cos(alpha) * np.sin(alpha) * W

    return (col, row)


def or2rot(dim, alpha):
    if (alpha > np.pi or alpha < -np.pi):
        warnings.warn('Are you using radians?')

    H = dim[0]
    W = dim[1]

    if alpha > 0:
        X = 1
        Y = W * np.sin(alpha)
    else:
        X = -H * np.sin(alpha)
        Y = 1

    LOC = np.matrix((X, Y))
    return LOC