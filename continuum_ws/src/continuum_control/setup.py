from setuptools import setup

package_name = 'continuum_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'control_node = continuum_control.control_node:main'
        ],
    },
)
