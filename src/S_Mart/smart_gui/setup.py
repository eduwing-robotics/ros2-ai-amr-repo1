from setuptools import find_packages, setup

package_name = 'smart_gui'

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
    maintainer_email='cks2904@naver.com',
    description='Smart Mart 관제 UI — PyQt5 + rclpy 관리자 대시보드',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'admin_gui = smart_gui.main:main',
        ],
    },
)
