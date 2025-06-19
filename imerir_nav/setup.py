import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'imerir_nav'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        ('share/ament_index/resource_index/packages',['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='corentin',
    maintainer_email='corentin.choisel@imerir.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'script_nav = imerir_nav.script_nav:main',
            'script_prof = imerir_nav.script_prof:main'
        ],
    },
)
