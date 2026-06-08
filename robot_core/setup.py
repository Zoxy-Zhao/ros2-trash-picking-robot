from setuptools import find_packages, setup

package_name = 'robot_core'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='zxy',
    maintainer_email='zxy2016567076@gmail.com',
    description='Main task scheduling and coordination node',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "main_node = robot_core.main_node:main"
        ],
    },
)
