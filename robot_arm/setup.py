from setuptools import find_packages, setup

package_name = 'robot_arm'

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
    description='Six-axis analytic IK and grasp planning with legacy four-axis support',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'arm_control = robot_arm.arm_control:main'
        ],
    },
)
