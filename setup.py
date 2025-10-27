from setuptools import setup

package_name = 'my_py_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='YOUR_NAME',
    maintainer_email='YOUR_EMAIL@example.com',
    description='Python package for ROS2 number publisher and counter nodes',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'number_publisher = my_py_pkg.number_publisher:main',
            'number_counter = my_py_pkg.number_counter:main',
        ],
    },
)
