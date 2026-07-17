from glob import glob

from setuptools import find_packages, setup

package_name = 'human_detector'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/models', ['models/human_best.pt']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='gustn',
    maintainer_email='difj3839@gmail.com',
    description='로봇 카메라 사람 감지 — YOLO로 사람 판별, /human_stop 발행 (nav2 BT 사람 게이트용)',
    license='MIT',
    entry_points={
        'console_scripts': [
            'human_detector = human_detector.human_detector_node:main',
        ],
    },
)
