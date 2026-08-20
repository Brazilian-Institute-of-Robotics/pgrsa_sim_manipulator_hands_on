from setuptools import setup
import os
from glob import glob

package_name = 'kuka_inverse_kinematics'

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
    description='Cinemática inversa KUKA KR70 R2100 — analítica e iterativa.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'ik_node    = kuka_inverse_kinematics.ik_node:main',
            'ik_plotter     = kuka_inverse_kinematics.ik_plotter:main',
            'ik_comparison  = kuka_inverse_kinematics.ik_comparison:main',
        ],
    },
)
