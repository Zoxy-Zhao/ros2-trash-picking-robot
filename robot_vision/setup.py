from setuptools import find_packages, setup

package_name = 'robot_vision'

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
    description='YOLO object detection and perspective-transform 3D localization',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_publisher = robot_vision.camera_publisher:main',
            'yolo_detection = robot_vision.yolo_detection:main',
            'perspective = robot_vision.perspective:main'
        ],
    },
)
