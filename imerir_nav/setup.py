from setuptools import find_packages, setup

package_name = 'imerir_nav'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ldl-ecole',
    maintainer_email='benoitlagasse@hotmail.com',
    description='Navigation between multiple positions',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'nav_node = imerir_nav.nav_node:main'
        ],
    },
)
