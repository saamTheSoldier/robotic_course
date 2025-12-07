from setuptools import setup

package_name = 'robot_local_localization'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/ekf_params.yaml']),
        ('share/' + package_name + '/launch', ['launch/hw2_localization.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='saam',
    maintainer_email='saam@example.com',
    description='ROS2 nodes for HW2 localization (prediction, measurement, ekf, test).',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'prediction_node = robot_local_localization.prediction_node:main',
            'measurement_node = robot_local_localization.measurement_node:main',
            'ekf_node = robot_local_localization.ekf_node:main',
            'test_node = robot_local_localization.test_node:main',
        ],
    },
)

