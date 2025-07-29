from setuptools import find_packages, setup

package_name = 'libo_waypoint_runner'

setup(
    name=package_name,
    version='0.0.0',
    # find_packages()를 사용하면 libo_waypoint_runner 폴더를 자동으로 패키지로 찾아줍니다.
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # config 파일을 설치 경로에 복사하도록 설정
        ('share/' + package_name + '/config', ['config/waypoints.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@todo.com',
    description='Waypoint runner for Libo robot',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 우리 노드를 'waypoint_runner'라는 실행 명령어로 등록합니다.
            # 경로가 libo_waypoint_runner 패키지 안의 waypoint_follower_node 파일을 가리키도록 수정되었습니다.
            'waypoint_runner = libo_waypoint_runner.waypoint_follower_node:main',
        ],
    },
)