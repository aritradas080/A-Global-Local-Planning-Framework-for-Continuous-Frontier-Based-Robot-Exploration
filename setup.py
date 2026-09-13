from setuptools import setup
import os
from glob import glob

package_name = 'nav_rtabmap'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Praveen Kumar Murali',
    maintainer_email='praveen.kumarmurali@udg.edu',
    description='RTAB-Map SLAM + RRT* + DWA + Frontier exploration',
    license='MIT',
    entry_points={
        'console_scripts': [
            'scan_relay        = nav_rtabmap.scan_relay:main',
            'scan_restamper    = nav_rtabmap.scan_restamper:main',
            'global_planner    = nav_rtabmap.global_planner:main',
            'dwa_planner       = nav_rtabmap.dwa_planner:main',
            'frontier_explorer = nav_rtabmap.frontier_explorer:main',
            'arm_tuck          = nav_rtabmap.arm_tuck:main',
        ],
    },
)
