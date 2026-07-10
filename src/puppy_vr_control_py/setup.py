from glob import glob

from setuptools import setup

package_name = 'puppy_vr_control_py'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ksyear',
    maintainer_email='ksyear@users.noreply.github.com',
    description='VR(Meta Quest) UDP 조종 + 카메라 UDP 전송 (파이썬판, C++ puppy_vr_control과 동일 동작)',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vr_udp_teleop = puppy_vr_control_py.vr_udp_teleop:main',
            'camera_udp_sender = puppy_vr_control_py.camera_udp_sender:main',
            'robot_status_sender = puppy_vr_control_py.robot_status_sender:main',
        ],
    },
)
