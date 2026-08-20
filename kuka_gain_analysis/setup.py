from setuptools import setup
import os
from glob import glob

package_name = 'kuka_gain_analysis'

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
    description='Análise de influência de ganhos na estabilidade e desempenho.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'gain_sweep       = kuka_gain_analysis.gain_sweep:main',
            'stability_monitor = kuka_gain_analysis.stability_monitor:main',
            'gain_report       = kuka_gain_analysis.gain_report:main',
        ],
    },
)
