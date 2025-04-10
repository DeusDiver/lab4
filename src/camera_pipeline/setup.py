from setuptools import setup
import os
from glob import glob

package_name = 'camera_pipeline'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ditt_navn',
    maintainer_email='din@email.no',
    description='Beskrivelse av pakken din',
    license='MIT',
    tests_require=['pytest'],
    include_package_data=True,  
    entry_points={
        'console_scripts': [
            'gaussian_blur = camera_pipeline.gaussian_blur:main'
        ],
    },
)
