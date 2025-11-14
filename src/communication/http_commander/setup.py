from setuptools import find_packages, setup
from glob import glob

package_name = 'http_commander'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        # launch 파일 설치
        ('share/' + package_name + '/launch', 
            glob('launch/*.launch.py')),

        # config 파일 설치
        ('share/' + package_name + '/config', 
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dk',
    maintainer_email='dk@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'http_client_node = http_commander.http_client_node:main'
        ],
    },
)
