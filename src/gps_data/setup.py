from setuptools import setup
import os
from glob import glob

package_name = 'gps_data'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools', 'pyserial', 'pynmeagps', 'matplotlib'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='ROS 2 GPS package publishing NavSatFix from NMEA serial data',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gps_node = gps_data.gps_node:main',
            'rtk_bridge_node = gps_data.rtk_bridge_node:main',
            'map_acc = gps_data.map_acc:main',
        ],
    },
)