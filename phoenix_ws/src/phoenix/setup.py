from setuptools import setup
from glob import glob
import os

package_name                  = 'phoenix'
mpc_submodule                 = 'phoenix/mpc'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name,mpc_submodule],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='imandralis@caltech.edu',
    description='Control package for ATMO/Phoenix, the Aerially Transforming Morphobot',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tilt_controller_hardware = phoenix.tilt_controller_hardware:main',
            'tilt_controller_sim = phoenix.tilt_controller_sim:main',
            'drive_controller_hardware = phoenix.drive_controller_hardware:main',
            'drive_controller_sim = phoenix.drive_controller_sim:main',
            'mpc_controller_hardware = phoenix.mpc_controller_hardware:main',
            'mpc_controller_sim = phoenix.mpc_controller_sim:main',
            'load_cell_test = phoenix.load_cell_test:main',
            'relay_mocap = phoenix.relay_mocap:main'
        ],
    },
)
