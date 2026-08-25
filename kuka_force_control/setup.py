from setuptools import setup
import os
from glob import glob

package_name = 'kuka_force_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='Controle de força com sensor F/T simulado no Gazebo.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'ft_sensor_sim       = kuka_force_control.ft_sensor_sim:main',
            'admittance_ctrl     = kuka_force_control.admittance_controller:main',
            'hybrid_force_pos    = kuka_force_control.hybrid_force_position:main',
            'contact_monitor     = kuka_force_control.contact_monitor:main',
        ],
    },
)
