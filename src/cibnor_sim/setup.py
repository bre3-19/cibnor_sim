from setuptools import find_packages, setup

package_name = 'cibnor_sim'
data_files = []
data_files.append(('share/ament_index/resource_index/packages', ['resource/' + package_name]))
## Config YAML
data_files.append(('share/' + package_name + '/config', ['config/cibnor_sim.yaml']))
## Launch files
data_files.append(('share/' + package_name + '/launch', ['launch/robot_launch.py']))
data_files.append(('share/' + package_name + '/launch', ['launch/robot_dist_sensors_launch.py']))
data_files.append(('share/' + package_name + '/launch', ['launch/robot_lidar_launch.py']))
data_files.append(('share/' + package_name + '/launch', ['launch/robot_odometry_launch.py']))
data_files.append(('share/' + package_name + '/launch', ['launch/robot_rviz_launch.py']))
## World files
data_files.append(('share/' + package_name + '/worlds', ['worlds/Greenhouse.wbt']))
## Resource files
data_files.append(('share/' + package_name + '/resource', ['resource/urdf_view.rviz']))
data_files.append(('share/' + package_name + '/resource', ['resource/lidar_scan.rviz']))
data_files.append(('share/' + package_name + '/resource', ['resource/robot.urdf']))
data_files.append(('share/' + package_name + '/resource', ['resource/robot_dist_sensor.urdf']))
data_files.append(('share/' + package_name + '/resource', ['resource/cibnor_sim.urdf']))
data_files.append(('share/' + package_name + '/resource', ['resource/cibnor_lidar_sensor.urdf']))
data_files.append(('share/' + package_name, ['package.xml']))

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='cibnor',
    maintainer_email='cibnor@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'robot_driver = cibnor_sim.webots_robot_driver:main',
            'obstacle_avoider = cibnor_sim.webots_obstacle_avoider:main',
            'odometry_publisher = cibnor_sim.odometry_publisher:main',
        ],
    },
)
