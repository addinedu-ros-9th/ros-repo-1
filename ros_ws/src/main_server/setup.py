from setuptools import find_packages, setup

package_name = 'main_server'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/weight_sensor.launch.py']),
    ],
    install_requires=['setuptools', 'pymysql>=1.1.0'],
    zip_safe=True,
    maintainer='robolee',
    maintainer_email='leesh20806@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'book_search_service = main_server.services.book_search_service:main',
            'robot_commander = main_server.services.robot_commander:main',
            'aladin_book_register = main_server.services.aladin_book_register:main',
            'task_manager = main_server.services.task_manager:main',
            'weight_subscriber = main_server.ros_communication.weight_subscriber:main',  # 무게 센서 구독자
            'weight_test_publisher = main_server.ros_communication.weight_test_publisher:main',  # 무게 테스트 퍼블리셔
            'led_status_publisher = main_server.led_status_publisher:main',  # LED 상태 퍼블리셔
            'camera_monitoring = main_server.camera_monitoring:main',  # 카메라 모니터링
        ],
    },
)
