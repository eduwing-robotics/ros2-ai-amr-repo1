import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'aruco_docking'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='codelab',
    maintainer_email='cks2904@naver.com',
    description='Smart Mart ArUco 도킹 — 인식(aruco_estimator) + 제어(dock_controller)',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'aruco_estimator = aruco_docking.aruco_estimator_node:main',
            'dock_controller = aruco_docking.dock_controller_node:main',
        ],
    },
)
