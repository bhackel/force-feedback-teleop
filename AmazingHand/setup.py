from setuptools import find_packages, setup

package_name = 'amazing_hand'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='user@todo.todo',
    description='AmazingHand ROS2 Python package for robotic hand control',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'demo = amazing_hand.AmazingHand_Demo:main',
            'demo_both = amazing_hand.AmazingHand_Demo_Both:main',
            'finger_test = amazing_hand.AmazingHand_FingerTest:main',
            'finger_middle_pos = amazing_hand.AmazingHand_Hand_FingerMiddlePos:main',
        ],
    },
)
