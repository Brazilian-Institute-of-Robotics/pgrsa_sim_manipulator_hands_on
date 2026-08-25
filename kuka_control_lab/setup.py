import os
from glob import glob

from setuptools import setup

package_name = 'kuka_control_lab'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='Estrategias de controle de manipuladores em malha fechada.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'strategy_sim     = kuka_control_lab.strategy_sim:main',
            'strategy_compare = kuka_control_lab.strategy_compare:main',
            'jtc_probe        = kuka_control_lab.jtc_probe:main',
        ],
    },
)
