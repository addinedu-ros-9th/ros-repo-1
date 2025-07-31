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
            'vision_manager = ai_service.nodes.vision_manager:main',
            'velocity_manager = ai_service.nodes.velocity_manager:main',
            'talker_manager = ai_service.nodes.talker_manager:main'            
        ],
    },
)
