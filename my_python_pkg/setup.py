from glob import glob
from setuptools import find_packages, setup

package_name = 'my_python_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),

        ('share/' + package_name,
            ['package.xml']),

        ('share/' + package_name + '/launch',
            ['launch/display.launch.py']),

        ('share/' + package_name + '/urdf',
            glob('urdf/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='febin',
    maintainer_email='febin@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            "febin=my_python_pkg.my_first_node:main",
            "pub_node=my_python_pkg.pub:main",
            "sub_node=my_python_pkg.sub:main",
            "lum=my_python_pkg.publisher:main",
            "sub=my_python_pkg.sub:main",
            "server_node=my_python_pkg.add_two_int:main",
            "client_node=my_python_pkg.client:main",
            "cmd_vel_serial=my_python_pkg.cmd_vel_serial:main",
            "esp32_driver=my_python_pkg.esp32_teleop_odometry_node:main",
            "odom_check=my_python_pkg.odom_check:main"
        ],
    },
)
