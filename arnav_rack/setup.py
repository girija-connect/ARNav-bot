import os
from glob import glob

from setuptools import setup

package_name = 'arnav_rack'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ARNAV maintainer',
    maintainer_email='p6384232@gmail.com',
    description='ARNAV rack detection and docking',
    license='MIT',
    entry_points={
        'console_scripts': [
            'scan_merger = arnav_rack.rack_nodes:scan_merger_main',
            'rack_detector = arnav_rack.rack_nodes:rack_detector_main',
            'jack_controller = arnav_rack.rack_nodes:jack_controller_main',
            'rack_docking = arnav_rack.rack_nodes:rack_docking_main',
            'rfid_reader_sim = arnav_rack.rack_nodes:rfid_reader_sim_main',
        ],
    },
)
