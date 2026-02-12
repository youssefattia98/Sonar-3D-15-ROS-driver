from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'sonar3d'

sonar3d_packages = find_packages(exclude=['test'])
wlsonar_packages = find_packages(where='wlsonar/src')

setup(
    name=package_name,
    version='0.0.0',
    packages=sonar3d_packages + wlsonar_packages,
    package_dir={
        'wlsonar': 'wlsonar/src/wlsonar',
    },
    include_package_data=True,
    package_data={
        'wlsonar.range_image_protocol': [
            '_proto/*.py',
            '_proto/*.pyi',
            '_proto/*.proto',
        ],
        'wlsonar': [
            'py.typed',
        ],
    },
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    zip_safe=True,
    maintainer='Water Linked',
    maintainer_email='support@waterlinked.com',
    description='TODO: Package description',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sonar_publisher = sonar3d.multicast_listener:main'
        ],
    },
)
