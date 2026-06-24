from setuptools import find_packages, setup

package_name = 'caddie_interaction'

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
    description='Voice and conversational command processing for WSO2 Caddie.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'voice_command_node = caddie_interaction.voice_command_node:main',
            'llm_command_node = caddie_interaction.llm_command_node:main',
        ],
    },
)
