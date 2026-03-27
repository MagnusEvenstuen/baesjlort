from setuptools import find_packages, setup

package_name = 'video_sender_used_for_testing'

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
    maintainer='gud',
    maintainer_email='143505668+MagnusEvenstuen@users.noreply.github.com',
    description='Package for sending video over ROS2',
    license='Apache-2.0',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'video_publisher = video_sender_used_for_testing.video_publisher:main',
        ],
    },
)