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
    maintainer='evafc003',
    maintainer_email='evaferdelacruz@gmail.com',
    description='Control de movimiento del robot NAO usando ROS2',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'nao_controller = moving_nao.nao_controller:main',
            'butterfly_swim = moving_nao.butterfly_swim:main',
            'moving_in_pattern = moving_nao.moving_in_pattern:main',
            'camina = moving_nao.camina:main',            
        ],
    },
    data_files=[
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share/ament_index/resource_index/packages'), ['resource/' + package_name]),
    ],
)

