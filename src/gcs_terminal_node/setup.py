from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['gcs_terminal_node'],
        # scripts=['scripts/gcs_node.py'],
    package_dir={'': 'src'}
)

setup(**d)