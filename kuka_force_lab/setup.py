import os
from glob import glob

from setuptools import setup

package_name = 'kuka_force_lab'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'models', 'contact_panel'),
         glob('models/contact_panel/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='Controle de forca e interacao com o ambiente.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'force_sim               = kuka_force_lab.force_sim:main',
            'force_compare           = kuka_force_lab.force_compare:main',
            'stiffness_sweep         = kuka_force_lab.stiffness_sweep:main',
            'ft_estimator            = kuka_force_lab.ft_estimator:main',
            'admittance_ctrl         = kuka_force_lab.admittance_ctrl:main',
            'contact_probe           = kuka_force_lab.contact_probe:main',
            'spawn_panel             = kuka_force_lab.spawn_panel:main',
            'enable_effort_interface = kuka_force_lab.enable_effort_interface:main',
        ],
    },
)
