import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'pod_serial_reader'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='sebas',
    maintainer_email='sebastiannewroblox@gmail.com',
    description='Reads the POD S3 CSV serial stream and displays/publishes it.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'serial_reader = pod_serial_reader.serial_reader_node:main',
            'limit_buzzer = pod_serial_reader.limit_buzzer_node:main',
        ],
    },
)
