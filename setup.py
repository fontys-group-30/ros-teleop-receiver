from setuptools import find_packages, setup
package_name = 'teleop-receiver'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/launch.py']),  # Add this line
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Tim',
    maintainer_email='386482@student.fontys.nl',
    description='Node for receiving data from teleop key',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'receiver_node = teleop_receiver.receiver_node:main',
            'static_tf_broadcaster = teleop_receiver.static_tf_broadcaster:main',
            'dynamic_tf_broadcaster = teleop_receiver.dynamic_tf_broadcaster:main',
        ],
    },
)