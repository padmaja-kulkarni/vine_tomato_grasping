## dependencies

python 2.7


## Issues

# cv2 setup


See [here](https://stackoverflow.com/questions/63346648/python-2-7-installing-opencv-via-pip-virtual-environment)
Python 2.7 is not supported anymore in opencv-python-4.3.0.38 the support was dropped since 4.3.0.36, see this issue.

The workaround I found was to install opencv-python version 4.2.0.32 (which is the latest supported for Python 2.7, see this for all releases) like this:

```
pip2 install opencv-python==4.2.0.32
```
