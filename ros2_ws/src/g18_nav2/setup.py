from setuptools import find_packages, setup
from glob import glob
from setuptools import find_packages, setup 
package_name = 'g18_nav2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
        data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/maps', glob('maps/*')),
    ],

    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='Nav2 configuration for the Group 18 pallet robot',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'wander = g18_nav2.wander:main',
            'go_to = g18_nav2.go_to:main',
            'spill_keepout = g18_nav2.spill_keepout:main',
            'cargo_manager = g18_nav2.cargo_manager:main',
        ],
    },
)
