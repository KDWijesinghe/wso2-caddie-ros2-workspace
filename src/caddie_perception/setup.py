from setuptools import find_packages, setup

package_name = 'caddie_perception'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Team Talos',
    maintainer_email='abdulr.23@cse.mrt.ac.lk',
    description='Perception nodes for WSO2 Caddie simulation.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'golf_ball_detector = caddie_perception.golf_ball_detector:main',
            'sensor_frame_normalizer = caddie_perception.sensor_frame_normalizer:main',
            'vlm_scene_context = caddie_perception.vlm_scene_context:main',
        ],
    },
)
