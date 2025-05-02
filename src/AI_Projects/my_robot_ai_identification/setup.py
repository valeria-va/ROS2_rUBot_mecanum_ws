from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'my_robot_ai_identification'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config_arg'), glob(os.path.join('config_arg', '*.*'))),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[py|xml]'))),
        (os.path.join('share', package_name, 'models'), glob(os.path.join('models', '*.*'))),
        (os.path.join('share', package_name, 'photos'), glob(os.path.join('photos', '*.*'))),
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
            'take_photo_exec = my_robot_ai_identification.take_photo:main',
            'takePhoto_detectSign_keras_exec = my_robot_ai_identification.takePhoto_detectSign_keras:main',
        ],
    },
)
