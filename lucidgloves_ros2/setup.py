from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'lucidgloves_ros2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, package_name), ['lucidgloves_ros2/config.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='bryce',
    maintainer_email='34104885+bhackel@users.noreply.github.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'lucidgloves_controller = lucidgloves_ros2.lucidgloves_controller:main',
        ],
    },
)
