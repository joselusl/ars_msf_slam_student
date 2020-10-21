from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
  packages=['ars_msf_slam'],
	package_dir={'': 'source'}
)

setup(**setup_args)

