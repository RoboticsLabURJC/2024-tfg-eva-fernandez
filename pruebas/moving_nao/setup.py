from setuptools import setup
import os
from glob import glob

package_name = 'moving_nao'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tu_nombre',
    maintainer_email='tu_email@example.com',
    description='Control de movimiento del robot NAO usando ROS2',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'nao_controller = moving_nao.nao_controller:main',
        ],
    },
    data_files=[
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share/ament_index/resource_index/packages'), ['resource/' + package_name]),
    ],
)

