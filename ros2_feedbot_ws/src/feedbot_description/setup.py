from setuptools import setup
import os
from glob import glob

package_name = 'feedbot_description'

setup(
    name=package_name,
    version='0.0.0',

    # 🔥 THIS WAS MISSING
    packages=[package_name],

    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*')),
    ],

    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dell',
    maintainer_email='dell@todo.todo',
    description='Feedbot robot description',
    license='Apache License 2.0',
    tests_require=['pytest'],

    entry_points={
        'console_scripts': [
            'step_test_joint = feedbot_description.step_test_joint:main',
            'step_test_all_joints = feedbot_description.step_test_all_joints:main',
        ],
    },
)
