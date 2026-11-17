import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'xbee_teleop'

setup(
    name=package_name,
    version='0.2.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='you@example.com',
    description='Gamepad teleop over XBee with a PyQt5 status GUI',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'tx_node = xbee_teleop.tx_node:main',
            'rx_node = xbee_teleop.rx_node:main',
            'gui = xbee_teleop.gui:main',
            'xbee_check = xbee_teleop.xbee_check:main',
        ],
    },
)
