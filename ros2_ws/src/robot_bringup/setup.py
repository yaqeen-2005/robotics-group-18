from glob import glob

from setuptools import setup

package_name = 'robot_bringup'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/resource', glob('resource/*.urdf')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Sirui Meng',
    maintainer_email='noreply@example.com',
    description='ROS 2 bridge for the Group 18 Webots pallet robot.',
    license='MIT',
    entry_points={'console_scripts': ['scan_normalizer = robot_bringup.scan_normalizer:main']},
)
