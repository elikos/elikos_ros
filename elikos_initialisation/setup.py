from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_parameters = generate_distutils_setup(
    packages=['elikos_initialisation'],
    scripts=[],
    package_dir={'': 'src'}
)

setup(**setup_parameters)
