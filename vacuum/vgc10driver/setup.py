from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'vgc10driver'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='alejandro.gonzalez@local.eurecat.org',
    maintainer_email='alejandro.gonzalez@eurecat.org',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vgc10_gripper_base = vgc10driver.vgc10_gripper_base:test',
            'vgc10_ros2_driver = vgc10driver.vgc10_ros2_driver:main',
        ],
    },
)
