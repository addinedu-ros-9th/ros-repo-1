from setuptools import find_packages, setup

package_name = 'kiosk'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/ui_files', ['kiosk/ui_files/main_window_kiosk.ui']),
        ('share/' + package_name + '/ui_files', ['kiosk/ui_files/book_detail_popup.ui']),
        ('share/' + package_name + '/ui_files', ['kiosk/ui_files/bookSearch.ui']),
        ('share/' + package_name + '/ui_files', ['kiosk/ui_files/book_corner_widget.ui']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robolee',
    maintainer_email='leesh20806@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'main_ui = kiosk.ui.main_window:main', # main_window.py의 main 함수를 main_ui라는 이름으로 실행할 수 있게 등록
            'taskrequest_test_tool = kiosk.ros_communication.taskrequest_test_tool:main', # taskrequest_test_tool 추가
        ],
    },
)
