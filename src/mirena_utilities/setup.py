from setuptools import find_packages, setup

package_name = 'mirena_utilities'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'opencv-python', 'open3d'],
    zip_safe=True,
    maintainer='colmenasio',
    maintainer_email='a.colmenero.fernandez@gmail.com',
    description='Utilities for debugging and other tasks on our simulator',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "remo_con = mirena_utilities.RemoCon:main"
        ],
    },
)
