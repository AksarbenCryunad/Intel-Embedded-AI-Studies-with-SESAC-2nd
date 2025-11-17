from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'autonomous'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         glob(os.path.join('autonomous', '*.launch.py'))),
        (os.path.join('share', package_name, 'config'),
         glob(os.path.join('config', '*.yaml'))),
        (os.path.join('share', package_name, 'resource'),
         glob(os.path.join('resource', '*.dae'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='intel',
    maintainer_email='cj6552@naver.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'auto_driving = autonomous.auto_driving:main',
        ],
    },
)
