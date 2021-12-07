from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup
  
d = generate_distutils_setup(
    packages=['microstrain_inertial_rqt'],
    package_dir={'': 'microstrain_inertial_rqt_common/src'},
)

setup(**d)
