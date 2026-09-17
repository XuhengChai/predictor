from setuptools import setup

package_name = 'zf_ego_angle_kinematics'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='zh',
    maintainer_email='xxx@zf.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ego_angle_kinematics = zf_ego_angle_kinematics.ego_angle_kinematics:main'
        ],
    },
)
