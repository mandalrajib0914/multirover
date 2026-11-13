from glob import glob

from setuptools import find_packages, setup

package_name = 'mrs_perception'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rajib-mandal',
    maintainer_email='linearwebsolution@gmail.com',
    description='Victim detection for the multirover robots',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'aruco_victim_node = mrs_perception.aruco_victim_node:main',
            # yolo_victim_node runs from the venv: .venv/bin/python -m mrs_perception.yolo_victim_node
        ],
    },
)
