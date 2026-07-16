from glob import glob

from setuptools import find_packages, setup

package_name = 'ai_detector'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/product_map.yaml']),
        # glob — launch 파일 추가 시 여기 손대는 걸 잊어 설치 누락되는 일 방지
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/models', ['models/best.pt']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='gustn',
    maintainer_email='difj3839@gmail.com',
    description='AI 입고 감지 — YOLO로 IN-1 게이트 물품 판별, /detection/inbound 발행',
    license='MIT',
    entry_points={
        'console_scripts': [
            'inbound_detector = ai_detector.inbound_detector:main',
            'outbound_detector = ai_detector.outbound_detector:main',
        ],
    },
)
