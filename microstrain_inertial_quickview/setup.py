from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup
  
d = generate_distutils_setup(
    packages=['microstrain_inertial_quickview'],
    package_dir={'': 'microstrain_inertial_quickview_common/src'},
)

setup(**d)
