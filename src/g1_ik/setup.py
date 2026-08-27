from setuptools import find_packages, setup

package_name = 'g1_ik'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='temo',
    maintainer_email='temomez123@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        	'g1_ik_controller = g1_ik.g1_ik_controller:main',
        	'g1_ik_controller_v2 = g1_ik.g1_ik_controller_v2:main',
        	'robot_arm_ik = g1_ik.robot_arm_ik:main',
        	'robot_arm_ik_v2 = g1_ik.robot_arm_ik_v2:main',
        	'weighted_moving_filter = g1_ik.weighted_moving_filter:main'
        ],
    },
)
