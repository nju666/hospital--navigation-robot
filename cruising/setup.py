from setuptools import setup
from glob import glob
import os

package_name = 'originbot_cruising'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    py_modules=[
        'originbot_cruising.cruise_node',
    ],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('originbot_cruising/launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='gogogo',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cruise_node = originbot_cruising.cruise_node:main',
        ],
    },
)
