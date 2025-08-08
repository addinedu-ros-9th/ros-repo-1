from setuptools import find_packages, setup

package_name = 'ai_service'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
            ['launch/ai_service.launch.py']),        
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='addinedu',
    maintainer_email='ajferhcz153@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'follower = ai_service.nodes.follower:main',
            'vision_manager = ai_service.nodes.vision_manager:main',
            'talker_manager = ai_service.nodes.talker_manager:main',
            'talker_manager2 = ai_service.nodes.talker_manager2:main',
            'test_voice_command_maker = ai_service.nodes.test_voice_command_maker:main', 
            'qr_scanner_node = ai_service.nodes.qr_scanner_node:main',
            'human_detector_node = ai_service.nodes.human_detector_node:main',
            'obstacle_detector_node = ai_service.nodes.obstacle_detector_node:main', 
            'ros2_bridege_node = ai_service.nodes.ros2_bridege_node:main',
            'vision_manager_assist = ai_service.nodes.vision_manager_assist:main',  
            'vision_manager_assist2 = ai_service.nodes.vision_manager_assist2:main', 
            'hand_gesture_detector_ros2_bridge = ai_service.nodes.hand_gesture_detector_ros2_bridge:main', 
            'vision_manager_assist3 = ai_service.nodes.vision_manager_assist3:main',
            'vision_manager_assist4 = ai_service.nodes.vision_manager_assist4:main',  
            'gesture_control_node = ai_service.nodes.gesture_control_node:main',

        ],
    },
)
