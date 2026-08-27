from setuptools import setup
from glob import glob

package_name = 'coverage_planner'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Prasiddh',
    maintainer_email='prasiddh@example.com',
    description='Boustrophedon coverage route planner for the strawberry tabletop AMR.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'coverage_planner_node = coverage_planner.coverage_planner_node:main',
        ],
    },
)
