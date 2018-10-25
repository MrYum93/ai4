from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['mavlink_lora'],
    scripts=['scripts/local_setpoint_pub.py','scripts/gcs_simple.py', 'scripts/gcs_keypress.py', 'scripts/param_list_get.py', 'scripts/mission_download.py', 'scripts/mission_set_current.py', 'scripts/req_data_stream.py'],
    package_dir={'': 'src'}
)

setup(**d)