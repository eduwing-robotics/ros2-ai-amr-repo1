from setuptools import find_packages, setup

package_name = 'robot_fsm'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/locations.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='gustn',
    maintainer_email='difj3839@gmail.com',
    description='S-Mart 로봇 FSM — 로봇 도메인에서 실행, Traffic Manager 연동 각진 주행',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_fsm = robot_fsm.robot_fsm:main',
        ],
    },
)
