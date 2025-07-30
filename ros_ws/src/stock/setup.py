from setuptools import setup
import os
import sys

package_name = 'stock'

# 가상환경 Python 경로 지정
venv_python = os.path.expanduser('~/venv/jazzy/bin/python')
if os.path.exists(venv_python):
    sys.executable = venv_python

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/ui', ['ui_files/stock_ui.ui']),
    ],
    install_requires=[
        'setuptools',
        'opencv-python>=4.5.0',
        'imutils>=0.5.4',
        'pyzbar>=0.1.8',
        'numpy>=1.19.0',
        'PyQt5>=5.14.0',
        'pymysql>=1.0.0',
        'requests>=2.25.0'
    ],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='Stock management package for book inventory',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'stock_gui = stock.stock_gui:main',
        ],
    },
) 