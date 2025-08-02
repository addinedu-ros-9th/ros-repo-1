from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'admin'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'ui'), glob('ui/*.ui')),
        (os.path.join('share', package_name, 'resource'), glob('resource/*')),  # 리소스 파일들 추가
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dan',
    maintainer_email='kimdaren2@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'debug_tool = admin.debug_tool:main',
            'debug_tool_qt = admin.debug_tool_qt:main',
            'admin_gui = admin.main_app:main',  # admin_gui 라는 이름으로 실행할 수 있도록 추가
        ],
    },
)
