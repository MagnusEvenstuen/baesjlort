from setuptools import find_packages, setup

package_name = 'image_compressor'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'setuptools',
        'opencv-python',
        'numpy',
    ],
    zip_safe=True,
    maintainer='gud',
    maintainer_email='143505668+MagnusEvenstuen@users.noreply.github.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'jpegls_compressor = image_compressor.compression_node:main',
            'jpegls_decompressor = image_compressor.decompression_node:main',
        ],
    },
)
