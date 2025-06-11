from setuptools import setup 
import os 
from glob import glob 
 
package_name = 'pm_swarm' 
 
setup( 
    name=package_name, 
    version='0.0.1', 
    packages=[package_name], 
    data_files=[ 
        ('share/' + package_name, ['package.xml']), 
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')), 
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')), 
    ], 
    install_requires=['setuptools'], 
    tests_require=['pytest'], 
    zip_safe=True, 
    maintainer='orchestrator', 
    maintainer_email='you@example.com', 
    description='Basic MAVLink to ROS 2 bridge node', 
    license='MIT', 
    entry_points={ 
        'console_scripts': [ 
            'mavlink_ros2_arm = mavlink_ros2_bridge.mavlink_ros2_arm:main', 
        ], 
    }, 
) 
