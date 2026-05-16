from setuptools import find_packages, setup

package_name = 'yolo_speaker'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mdyesley',
    maintainer_email='you@example.com',
    description='Speaks YOLO detections and shows annotated video.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'announce_node = yolo_speaker.announce_node:main',
            'debug_viewer_node = yolo_speaker.debug_viewer_node:main',
        ],
    },
)
