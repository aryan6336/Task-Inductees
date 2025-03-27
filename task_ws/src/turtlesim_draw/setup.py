from setuptools import find_packages, setup

package_name = 'turtlesim_draw'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nambiar',
    maintainer_email='nambiar@todo.todo',
    description='A ROS 2 package to draw shapes using Turtlesim',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "turtle_draw = turtlesim_draw.code:main",
        ],
    },
)

