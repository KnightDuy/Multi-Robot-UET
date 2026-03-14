from setuptools import find_packages, setup

package_name = 'odom_encoder'

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
    maintainer='duy',
    maintainer_email='duydz151204@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'odom_node = odom_encoder.odom_node:main',
            'fake_vel_node = odom_encoder.fake_vel_node:main',
            'fake_imu_node = odom_encoder.fake_imu_node:main',
            'imu_node = odom_encoder.imu_node:main',
        ],
    },
)
