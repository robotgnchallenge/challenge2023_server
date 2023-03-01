from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['hybrid_mpc_local_planner'],
    package_dir={'': 'scripts'}
)

setup(**d)
