from glob import glob

from setuptools import find_packages, setup

package_name = 'mrs_exploration'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rajib-mandal',
    maintainer_email='linearwebsolution@gmail.com',
    description='Frontier exploration for the multirover robots',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'exploration_monitor = mrs_exploration.exploration_monitor:main',
            'coordinator = mrs_exploration.coordinator:main',
        ],
    },
)
