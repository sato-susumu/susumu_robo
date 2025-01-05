import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'susumu_robo'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'param'), glob(os.path.join('param', '*.yaml'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.rviz'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Sato Susumu',
    maintainer_email='75652942+sato-susumu@users.noreply.github.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "laserscan_filter_node = susumu_robo.laserscan_filter_node:main",
            "twist_filter_node = susumu_robo.twist_filter_node:main",
            "tenkey_controller = susumu_tenkey_controller.tenkey_controller:main",
            "led_controller_node = susumu_robo.nodes.led_controller_node:main",
        ],
    },
)
