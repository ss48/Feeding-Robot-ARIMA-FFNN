import os
from glob import glob
from setuptools import setup

package_name = 'feedbot_fusion'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools', 'numpy'],
    zip_safe=True,
    maintainer='Shabnam Sadeghi Esfahlani',
    maintainer_email='shabnam.sadeghi-esfahlani@aru.ac.uk',
    description='Feedbot fusion, prediction, and control nodes with ARIMA-FFNN',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vision_node = feedbot_fusion.vision_node:main',
            'fusion_node = feedbot_fusion.fusion_node:main',
            'force_node = feedbot_fusion.force_node:main',
            'speed_controller = feedbot_fusion.speed_controller:main',
            'feeding_fsm = feedbot_fusion.feeding_fsm_node:main',
            'arima_ffnn = feedbot_fusion.arima_ffnn_node:main',
            'fuzzy_controller = feedbot_fusion.fuzzy_controller_node:main',
            'mouth_animator = feedbot_fusion.mouth_animator_node:main',
            'sonar_bridge = feedbot_fusion.sonar_bridge_node:main',
            'teensy_bridge = feedbot_fusion.teensy_bridge_node:main',
            'face_node = feedbot_fusion.face_node:main',
        ],
    },
)
