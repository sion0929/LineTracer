from setuptools import find_packages, setup

package_name = 'line_follower_gcs'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    package_data={package_name: ['fonts/*']},
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    extras_require={'test': ['pytest']},
    zip_safe=True,
    maintainer='sion',
    maintainer_email='202042036@cbnu.ac.kr',
    description='Development-only GCS nodes for the line follower rover',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'gcs_recorder = line_follower_gcs.gcs_recorder:main',
            'gcs_bridge = line_follower_gcs.gcs_bridge:main',
            'gcs_command = line_follower_gcs.gcs_command:main',
            'gcs_mission_manager = line_follower_gcs.gcs_mission_manager:main',
        ],
    },
)
