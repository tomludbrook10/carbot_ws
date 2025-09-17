from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'acml_with_map'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.py'))),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tom',
    maintainer_email='tom@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'initial_pose_publisher = acml_with_map.initial_pose_publisher:main',
            'amcl_lifecycle_manager = acml_with_map.amcl_lifecycle_managerv1:main',
            'amcl_test = acml_with_map.amcl_test:main'
        ],
    },
)
