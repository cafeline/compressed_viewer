from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'compressed_viewer'

setup(
    name='compressed_viewer',
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include launch files
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
        # Include config files
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
        # Include RViz config files
        (os.path.join('share', package_name, 'rviz'),
            glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools', 'numpy'],
    zip_safe=True,
    maintainer='ryo',
    maintainer_email='s24s1040du@s.chibakoudai.jp',
    description='Viewer for compressed point cloud data with RViz2 visualization',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'compressed_viewer_node = compressed_viewer.compressed_viewer_node:main',
            'compressed_viewer = compressed_viewer.compressed_viewer:main'
        ],
    },
)
