# ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD
from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
    packages=['spawn_truss', 'spawn_truss.state_machine'],
    package_dir={'': 'src'}
)
setup(**setup_args)
