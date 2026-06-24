from setuptools import find_packages, setup

package_name = 'caddie_control'

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
    description='Control helpers for the simulated Go2 caddie.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'go2_gait_animator = caddie_control.go2_gait_animator:main',
            'go2_velocity_limiter = caddie_control.go2_velocity_limiter:main',
            'hidden_wheel_joint_state_publisher = caddie_control.hidden_wheel_joint_state_publisher:main',
            'unitree_sportmode_bridge = caddie_control.unitree_sportmode_bridge:main',
        ],
    },
)
