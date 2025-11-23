from setuptools import setup

package_name = 'camera_publisher'

setup(
    name=package_name,
    version='0.0.0',
    packages=['python'],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/msg', ['msg/Person_marker.msg', 'msg/Persons.msg']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='fabko',
    maintainer_email='fabko@todo.todo',
    description='Camera publisher with Python node',
    license='TODO',
    entry_points={
        'console_scripts': [
            'camera_publisher = python.camera_pub:main',  # main Funktion in camera_pub.py
            'shm_writer = python.shm_writer_test:main',  # main Funktion in camera_pub.py
        ],
    },
)
