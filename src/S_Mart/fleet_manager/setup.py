from setuptools import find_packages, setup

package_name = 'fleet_manager'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='codelab',
    maintainer_email='rpehd2904@gmail.com',
    description='Smart Mart Fleet Manager — 로봇 배정 + DB 상태 전환 ROS2 노드',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fleet_manager = fleet_manager.fleet_manager_node:main',
        ],
    },
)
