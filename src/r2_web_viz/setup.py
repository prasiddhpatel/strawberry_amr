import os
from glob import glob
from setuptools import setup

package_name = 'r2_web_viz'


def web_data_files():
    """Preserve web/'s directory structure under share/r2_web_viz/web/...
    ament_python's data_files only takes flat (dest, [files]) pairs, so a
    static-asset tree with subdirectories (css/, js/, js/vendor/) has to be
    walked and flattened into one pair per directory, not one glob."""
    pairs = []
    for root, _dirs, files in os.walk('web'):
        if not files:
            continue
        dest = os.path.join('share', package_name, root)
        srcs = [os.path.join(root, f) for f in files]
        pairs.append((dest, srcs))
    return pairs


setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ] + web_data_files(),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Prasiddh',
    maintainer_email='prasiddh@example.com',
    description='Browser-based live visualization (2D map, 3D point cloud, camera) for the R2 robot.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'map_accumulator_node = r2_web_viz.map_accumulator_node:main',
        ],
    },
)
