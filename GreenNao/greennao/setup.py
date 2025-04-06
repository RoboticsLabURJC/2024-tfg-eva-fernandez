from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'greennao'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='evafc003',
    maintainer_email='evaferdelacruz@gmail.com',
    description='This workspace is my TFG\'s home',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'movement_interpreter = greennao.movement_interpreter:main',         
        ],
    },
    data_files=[
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share/ament_index/resource_index/packages'), ['resource/' + package_name]),
    ],
)
