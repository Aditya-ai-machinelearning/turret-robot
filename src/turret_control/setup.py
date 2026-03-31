from setuptools import find_packages, setup

package_name = 'turret_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/turret_control']),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
            ['launch/turret.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu22',
    maintainer_email='ubuntu22@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'pid_node = turret_control.pid_node:main',
            'sc15_node = turret_control.sc15_node:main',
            'fire_node = turret_control.fire_node:main',
            'imu_node = turret_control.imu_node:main',
            'dashboard_node = turret_control.dashboard_node:main',
        ],
    },
)