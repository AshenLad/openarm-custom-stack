from setuptools import find_packages, setup

package_name = 'openarm_learning'

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
    maintainer='ashenlad',
    maintainer_email='ashenlad@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'joint_monitor = openarm_learning.joint_monitor:main',
            'tf_monitor = openarm_learning.tf_monitor:main',
            'fake_camera_broadcaster = openarm_learning.fake_camera_broadcaster:main',
            'trajectory_action_client = openarm_learning.trajectory_action_client:main',
            ],
    },
)
