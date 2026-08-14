from setuptools import find_packages, setup

package_name = 'g1_odom_tools'

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
    maintainer='jscott',
    maintainer_email='raysco420@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'imu_frameflip = g1_odom_tools.imu_frameflip:main',
            'lio_to_base_broadcaster = g1_odom_tools.lio_to_base_broadcaster:main',
            'floor_level_anchor = g1_odom_tools.floor_level_anchor:main',
        ],
    },
)
