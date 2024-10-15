from setuptools import setup

package_name = 'my_gazebo_controller'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Gazebo Controller Example',
    entry_points={
        'console_scripts': [
            'gazebo_controller = my_gazebo_controller.controller:main',
        ],
    },
)
