from setuptools import find_packages, setup
from glob import glob

package_name = 'grpc_commander'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test'], include=[
        'grpc_commander',
        'grpc_commander.*',
        ]),
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
            'grpc_client_node = grpc_commander.grpc_client_node:main'
        ],
    },
)
