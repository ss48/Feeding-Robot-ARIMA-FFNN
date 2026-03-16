from setuptools import setup

package_name = 'feedbot_sensors'

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
    maintainer='dell',
    maintainer_email='dell@todo.todo',
    description='Sensor hub for feedbot',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sensor_hub = feedbot_sensors.sensor_hub:main',
        ],
    },
)

