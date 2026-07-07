import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'vhit_elac_tester'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join("share", package_name, "launch"),
            glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"),
            glob("config/*")),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='riky',
    maintainer_email='riccardo.valtorta@vhit-weifu.com',
    description='ELAC communication tester using joint_trajectory_controller topic commands.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'elac_tester_node = vhit_elac_tester.elac_tester_node:main'
        ],
    },
)
