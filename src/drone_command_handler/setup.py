from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['drone_command_handler'],
    scripts=['scripts/local_setpoint_manual_service_server.py','scripts/test_local_position_publisher.py'],
    package_dir={'': 'src'}
)

setup(**d)