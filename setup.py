import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'dexi_gpio'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Dennis Baldwin',
    maintainer_email='db@droneblocks.io',
    description='DEXI GPIO control package for Raspberry Pi GPIO pins',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gpio_writer_service = dexi_gpio.gpio_writer_service:main',
            'gpio_reader = dexi_gpio.gpio_reader:main',
        ],
    },
)