from setuptools import setup

package_name = 'drones_as2_unizar'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='David Morilla-Cabello',
    maintainer_email='davidmorillacabello@gmail.com',
    description='PKeep pose node for MAVROS using ROS 2',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'keep_pose = drones_as2_unizar.keep_pose:main',
        ],
    },
)