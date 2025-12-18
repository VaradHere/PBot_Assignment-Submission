from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'pbot_fleet'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='varadhere',
    maintainer_email='varad.sandeep.desai@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fleet_manager = pbot_fleet.fleet_manager_node:main',
            'interactive_waypoint_selector = pbot_fleet.order_waypoint_node:main',
        ],
    },
)
