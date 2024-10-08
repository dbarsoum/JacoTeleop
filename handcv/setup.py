from setuptools import find_packages, setup

package_name = 'handcv'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config',
         ['config/handcv.rviz', 'config/hand_landmarker.task', 'config/gesture_recognizer.task', 'config/high_density_preset.json']),
        ('share/' + package_name + '/launch', ['launch/camera.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Graham Clifford',
    maintainer_email='gclifford@u.northwestern.edu',
    description='A package implementing computer vision with the\
        Intel Realsense D435i Camera to recognize and find the 3D location of a human hand.',
    license='APLv2',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'handcv = handcv.handcv:main'
        ],
    },
)
