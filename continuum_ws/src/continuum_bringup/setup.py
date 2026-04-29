from setuptools import setup
import os
from glob import glob

package_name = 'continuum_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='you',
    maintainer_email='you@todo.com',
    description='Bringup package',
    license='MIT',

    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # 🔥 THIS LINE IS THE FIX
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.launch.py')),
    ],

    entry_points={
        'console_scripts': [],
    },
)