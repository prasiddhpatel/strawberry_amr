from setuptools import setup
from glob import glob

package_name = 'teleop_ps2'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Prasiddh',
    maintainer_email='prasiddh@example.com',
    description='teleop_ps2 for the strawberry tabletop AMR.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'joystick_node = teleop_ps2.joystick_node:main',
        ],
    },
)
