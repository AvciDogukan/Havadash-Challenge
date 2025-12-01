from setuptools import setup
import os
from glob import glob

package_name = 'havadash_bridge'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Launch dosyalarini install dizinine kopyalamak icin gereken satir:
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='Havadash Firmware Engineering Challenge',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mock_publisher = havadash_bridge.mock_publisher:main',
            'mqtt_bridge = havadash_bridge.mqtt_bridge:main',
        ],
    },
)
