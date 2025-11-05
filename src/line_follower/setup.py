from setuptools import find_packages, setup

package_name = 'line_follower'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
data_files=[
    ('share/ament_index/resource_index/packages',
        ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    ('share/' + package_name + '/launch', ['launch/launcher.py']),  
],

    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ever',
    maintainer_email='ever.alcaraz@cetys.edu.mx',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'inverse_kinematics = line_follower.control.inverse_kinematics:main',
            'lidar_reader = line_follower.control.lidar_reader:main',
            'line_PID = line_follower.control.line_PID:main',
            'vel_input = line_follower.control.vel_input:main',

            'prueba = line_follower.microcontrollers.prueba:main',
            'serial_controller = line_follower.microcontrollers.serial_controller:main',
            'wheel_velocity_PID = line_follower.microcontrollers.wheel_velocity_PID:main',
            

        ],
    },
)
