from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'my_robot_nav_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[py|xml]'))), # Afegit per a la carpeta launch
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'nav_target_exec = my_robot_nav_control.nav_target:main', # Canviat per al nou nom del fitxer
            'my_robot_nav_target1_exec = my_robot_nav_control.simple_commander_nav_target1:main',
        ],
    },
)