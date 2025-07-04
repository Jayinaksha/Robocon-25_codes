from setuptools import setup
import os

package_name = 'joy_serial_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='Jayinaksha',
    maintainer_email='jayinaksha_2302cm08@iitp.ac.in',
    description='Bridge /joy → ESP via serial',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'joy_to_serial = joy_serial_bridge.joy_to_serial:main',
        ],
    },
    data_files=[
        # ament index marker
        (
            'share/ament_index/resource_index/packages',
            ['resource/joy_serial_bridge']
        ),
        # install package.xml
        (
            os.path.join('share', package_name),
            ['package.xml']
        ),
    ],
)