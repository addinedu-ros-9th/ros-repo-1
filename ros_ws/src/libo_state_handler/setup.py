from setuptools import find_packages, setup

package_name = 'libo_state_handler'

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
    entry_points={
        'console_scripts': [
            'state_handler_node = libo_state_handler.state_handler_node:main'
        ],
    },
)
