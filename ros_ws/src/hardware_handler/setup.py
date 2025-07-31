from setuptools import find_packages, setup

package_name = 'hardware_handler'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
            ['launch/hardware_handler.launch.py']),        
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='addinedu',
    maintainer_email='ajferhcz153@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'io_controller_node = hardware_handler.nodes.io_controller_node:main',
            'cam_sender = hardware_handler.nodes.cam_sender:main',
            'mic_streamer = hardware_handler.nodes.mic_streamer:main',
            'speaker_node = hardware_handler.nodes.speaker_node:main',
            'rgb_led_controller = hardware_handler.nodes.rgb_led_controller:main',
            'heartbeat_sender = hardware_handler.nodes.heartbeat_sender:main',
        ],
    },
)
