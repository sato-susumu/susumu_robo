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
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')) + glob(os.path.join('launch', '*.sh'))),
        (os.path.join('share', package_name, 'param'), glob(os.path.join('param', '*.yaml'))),
        (os.path.join('share', package_name, 'mapping'), glob(os.path.join('mapping', '*.md')) + glob(os.path.join('mapping', '*.ini'))),
        (os.path.join('share', package_name, 'mapping/config'), glob(os.path.join('mapping', 'config', '*.json'))),
        (os.path.join('share', package_name, 'mapping/config/d435'), glob(os.path.join('mapping', 'config', 'd435', '*'))),
        (os.path.join('share', package_name, 'mapping/config/d455'), glob(os.path.join('mapping', 'config', 'd455', '*'))),
        (os.path.join('share', package_name, 'mapping/config/kinect'), glob(os.path.join('mapping', 'config', 'kinect', '*'))),
        (os.path.join('share', package_name, 'mapping/config/l515'), glob(os.path.join('mapping', 'config', 'l515', '*'))),
        (os.path.join('share', package_name, 'mapping/config/livox'), glob(os.path.join('mapping', 'config', 'livox', '*'))),
        (os.path.join('share', package_name, 'mapping/config/os0'), glob(os.path.join('mapping', 'config', 'os0', '*'))),
        (os.path.join('share', package_name, 'mapping/config/zed2i'), glob(os.path.join('mapping', 'config', 'zed2i', '*'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.rviz')) + glob(os.path.join('config', '*.yaml'))),
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
            "led_controller_node = susumu_robo.led_controller_node:main",
        ],
    },
)
