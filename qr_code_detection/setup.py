import os
from setuptools import setup
from glob import glob

package_name = 'qr_code_detection'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 仅保留这一行：launch文件只安装到launch子目录（核心！）
        ('share/' + package_name + '/launch', glob("launch/*.launch.py")),
        # model文件安装配置（保留）
        ('share/' + package_name+'/model', glob("model/*.*"))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='QR code detection package for ROS2',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'qr_detection_node = qr_code_detection.qr_detection_node:main',
            'qr_detection_fast_candidate = qr_code_detection.qr_detection_fast_candidate:main',
            'qr_detection_latest_wechat = qr_code_detection.qr_detection_latest_wechat:main',
            'qr_detection_opencv = qr_code_detection.qr_detection_opencv:main',
            'oled_text_renderer = qr_code_detection.oled_text_renderer:main',
        ],
    },
)
