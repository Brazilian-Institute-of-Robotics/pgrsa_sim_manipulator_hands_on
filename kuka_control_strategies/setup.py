from setuptools import setup
import os
from glob import glob

package_name = 'kuka_control_strategies'

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
    description='Comparação de estratégias de controle: local, centralizado, espaço operacional.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'local_pd_controller    = kuka_control_strategies.local_pd_controller:main',
            'computed_torque        = kuka_control_strategies.computed_torque:main',
            'operational_space      = kuka_control_strategies.operational_space:main',
            'strategy_benchmark     = kuka_control_strategies.strategy_benchmark:main',
        ],
    },
)
