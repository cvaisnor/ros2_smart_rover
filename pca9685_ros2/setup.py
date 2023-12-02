from setuptools import setup

package_name = 'pca9685_ros2'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
                ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Chris Vaisnor',
    maintainer_email='chrisvaisnor@gmail.com',
    description='ROS2 interface for the PCA9685 servo controller',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pca9685 = pca9685_ros2.pca9685_node:main'
        ],
    },
)