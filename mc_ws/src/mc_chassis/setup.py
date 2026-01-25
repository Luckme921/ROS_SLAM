from setuptools import find_packages, setup
from glob import glob  

package_name = 'mc_chassis'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools', 'pyserial', 'numpy'],
    zip_safe=True,
    maintainer='pi',
    maintainer_email='pi@example.com',
    description='Mecanum car controller with standard cmd_vel (ROS2 Humble)',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'serial_comm_node=mc_chassis.serial_comm_node:main',
            'odometry_publisher=mc_chassis.odometry_pub_node:main',
            'cmd_vel_keyboard=mc_chassis.cmd_vel_keyboard:main',
            'chassis_controller=mc_chassis.chassis_controller:main',
            'wheel_joint_states_publisher=mc_chassis.wheel_joint_states_publisher:main',
            'sound_vel=mc_chassis.sound_vel:main',
        ],
    },
)