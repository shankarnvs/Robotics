from setuptools import setup
import os
from glob import glob

package_name = 'continuum_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='you',
    maintainer_email='you@todo.com',
    description='Continuum control node',
    license='MIT',

    # 🔥 THIS PART WAS MISSING
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],

    entry_points={
        'console_scripts': [
            'control_node = continuum_control.control_node:main'
        ],
    },
)
