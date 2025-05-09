from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['SGFlightKitCore'],
    py_modules=['SGVideo'],
    package_dir={'': 'src'}
)

setup(**d)
