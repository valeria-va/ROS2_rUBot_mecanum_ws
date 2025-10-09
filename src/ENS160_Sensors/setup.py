from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'ENS160_Sensors'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='vvalleay94@alumnes.ub.edu',
    description='TODO: Package description',
    license='Apache License 2.0', # improved license
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ENS160_Sensors_exec = ENS160_Sensors.ENS160_Sensors:main',
        ],
    },
)
