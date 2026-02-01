from setuptools import setup

package_name = 'pidog_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='PiDog Developer',
    maintainer_email='your_email@example.com',
    description='PiDog control package - motion controller and dashboard',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'motion_node = pidog_control.motion_node:main',
            'dashboard = pidog_control.pidog_dashboard:main',
        ],
    },
)
