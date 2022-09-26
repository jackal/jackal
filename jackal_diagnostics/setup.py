from glob import glob

import os

from setuptools import setup

package_name = 'jackal_diagnostics'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rkreinin',
    maintainer_email='rkreinin@clearpathrobotics.com',
    description='Jackal Diagnostics',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'diagnostics_updater = jackal_diagnostics.diagnostics_updater:main'
        ],
    },
)
