from setuptools import find_packages, setup

package_name = 'bartender_test'

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
    maintainer='fastcampus',
    maintainer_email='fastcampus@todo.todo',
    description='Custom Doosan robot control nodes',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gripper = bartender_test.gripper_command:main',
            'pose = bartender_test.pose:main',
            'movej = bartender_test.movej:main',
            'movec = bartender_test.movec:main',
            'action = bartender_test.action:main',
            'check_tcp = bartender_test.check_tcp:main',
            'monitor = bartender_test.monitor:main',
        ],
    },
)
