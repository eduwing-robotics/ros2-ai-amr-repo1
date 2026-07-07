from setuptools import find_packages, setup

package_name = 'traffic_manager'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/graph', ['graph/nodes.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='codelab',
    maintainer_email='rpehd2904@gmail.com',
    description='S-Mart Traffic Manager — 노드/엣지 그래프 경로 생성(다익스트라) + 예약/교착 조율',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'traffic_node = traffic_manager.traffic_node:main',
        ],
    },
)
