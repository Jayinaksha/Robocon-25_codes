from setuptools import setup

package_name = 'basketball_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['launch/basketball_robot.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Basketball robot package',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'basketball_detector = basketball_robot.basketball_detector:main',
            'smart_drive_bridge = basketball_robot.smart_drive_bridge:main',
            'camera_display_node = basketball_robot.camera_display_node:main',
            'realsense_basket_detector = basketball_robot.realsense_basket_detector:main',
        ],
    },
)