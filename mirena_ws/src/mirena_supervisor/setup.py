from setuptools import setup
from glob import glob
import os

package_name = 'mirena_supervisor'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # The two entry points, and one launch file per mission. The supervisor
        # starts the latter itself, through the launch API, when the AS selects
        # a mission -- which is why they ship with this package.
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.rviz')),
        ('share/' + package_name + '/launch/missions', glob('launch/missions/*.launch.py')),
    ],
    install_requires=[
        'setuptools',
        'numpy',
        'scipy',
        'matplotlib'
    ],
    zip_safe=True,
    maintainer='Ozuba',
    maintainer_email='ozuba@ozuba.es',
    description='Mission supervisor: runs one mission launch file per run',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mirena_supervisor = mirena_supervisor.mirena_supervisor:main',
        ],
    },
)