from setuptools import setup

package_name = 'ntrip_client'

setup(
    name=package_name,
    version='1.4.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/ntrip_client_launch.py',
            'launch/ntrip_serial_device_launch.py',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Rob Fisher',
    maintainer_email='rob.fisher@parker.com',
    description='NTRIP client that will publish RTCM corrections to a ROS topic, and optionally subscribe to NMEA messages to send to an NTRIP server',
    license='MIT',
    entry_points={
        'console_scripts': [
            'ntrip_ros = ntrip_client.ntrip_ros:main',
            'ntrip_serial_device_ros = ntrip_client.ntrip_serial_device_ros:main',
        ],
    },
)
