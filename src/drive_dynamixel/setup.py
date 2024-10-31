from setuptools import find_packages, setup

package_name = 'drive_dynamixel'

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
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dynamixel_controller = drive_dynamixel.dynamixel_controller:main',
            'test_pub = drive_dynamixel.test_pub:main',
            'test_sub = drive_dynamixel.test_sub:main',
            'stop = drive_dynamixel.stop:main', 
        ],
    },
)
