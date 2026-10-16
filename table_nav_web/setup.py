from glob import glob
from setuptools import find_packages, setup

package_name = 'table_nav_web'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/web', glob('web/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='febin',
    maintainer_email='febin@todo.todo',
    description='Phone/web page to send the robot to saved places with Nav2.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'web_server = table_nav_web.web_server:main',
        ],
    },
)
