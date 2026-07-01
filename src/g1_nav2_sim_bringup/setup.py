from glob import glob
from setuptools import find_packages, setup

package_name = 'g1_nav2_sim_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/sim_bringup_launch.py']),
        ('share/' + package_name + '/config', ['config/slam_toolbox_params.yaml', 'config/nav2_params.yaml', 'config/g1_bringup.rviz']),
        ('share/' + package_name + '/worlds', ['worlds/cafe_no_people.world']),
        ('share/' + package_name + '/models/cafe_table', glob('models/cafe_table/model*')),
        ('share/' + package_name + '/models/cafe_table/meshes', glob('models/cafe_table/meshes/*')),
        ('share/' + package_name + '/models/cafe_table/materials/textures', glob('models/cafe_table/materials/textures/*'))
        
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jscott',
    maintainer_email='jscott@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        ],
    },
)
