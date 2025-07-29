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
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='addinedu',
    maintainer_email='ajferhcz153@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'io_controller_node = nodes.io_controller_node:main',
            'depth_cam_sender = nodes.depth_cam_sender:main',
            'mic_streamer = nodes.mic_streamer:main',
            'speaker_node = nodes.speaker_node:main',
            'rgb_led_controller = nodes.rgb_led_controller:main',            
        ],
    },
)
