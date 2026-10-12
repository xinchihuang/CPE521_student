from glob import glob
from setuptools import find_packages, setup

setup(
    name='rawseeds_slam',
    version='1.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/rawseeds_slam']),
        ('share/rawseeds_slam', ['package.xml']),
        ('share/rawseeds_slam/launch', glob('launch/*.py')),
        ('share/rawseeds_slam/config', glob('config/*.yaml')),
        ('share/rawseeds_slam/rviz', glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Project maintainer',
    maintainer_email='maintainer@example.com',
    description='Minimal Rawseeds SLAM student framework',
    license='MIT',
    entry_points={'console_scripts': [
        'trajectory = rawseeds_slam.trajectory:main',
        'slam_mapper = rawseeds_slam.student_node:main',
    ]},
)
