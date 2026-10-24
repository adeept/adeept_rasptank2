from setuptools import find_packages, setup

package_name = 'rasptank_hw'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='gbellido',
    maintainer_email='gbellido@mondragon.edu',
    description='RaspTank hardware nodes.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'base_node = rasptank_hw.base_node:main',
            'arm_node = rasptank_hw.arm_node:main',
            'sensors_node = rasptank_hw.sensors_node:main',
            'camera_node = rasptank_hw.camera_node:main',
            'motor_watchdog_node = rasptank_hw.motor_watchdog_node:main',
            'governor_node = rasptank_hw.governor_node:main',
        ],
    },
)
