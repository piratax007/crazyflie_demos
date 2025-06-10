from setuptools import find_packages, setup

package_name = 'crazyflie_demos'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Fausto',
    maintainer_email='piratax007@protonmail.ch',
    description='ROS2 nodes for testing Crazyflie drones',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'crazyflie_teleop = crazyflie_demos.crazyflie_teleop:main',
            'crazyflie_NN_controller = crazyflie_demos.crazyflie_NN_controller:main',
        ],
    },
)
