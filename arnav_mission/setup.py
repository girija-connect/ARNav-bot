import os
from glob import glob

from setuptools import setup

package_name = 'arnav_mission'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ARNAV maintainer',
    maintainer_email='p6384232@gmail.com',
    description='ARNAV mission commands',
    license='MIT',
    entry_points={
        'console_scripts': [
            'move_rack = arnav_mission.mission:move_rack_main',
            'run_tasks = arnav_mission.mission:run_tasks_main',
            'goto = arnav_mission.mission:goto_main',
            'rfid_cmd = arnav_mission.mission:rfid_cmd_main',
            'show_layout = arnav_mission.mission:show_layout_main',
        ],
    },
)
