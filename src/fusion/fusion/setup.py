from setuptools import setup, find_packages
import os
from glob import glob
import sys

package_name = 'fusion'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=["test"]),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # (os.path.join(
        #     'lib', f'python{sys.version_info.major}.{sys.version_info.minor}',
        #     'site-packages', package_name, 'ego_state_package',
        #     'params'), glob(os.path.join('params', '*.yaml'))),
        # (os.path.join(
        #     'lib', f'python{sys.version_info.major}.{sys.version_info.minor}',
        #     'site-packages', package_name, 'ego_state_package',
        #     'params'), glob(os.path.join('params', '*.xlsx'))),
        # (os.path.join(
        #     'lib', f'python{sys.version_info.major}.{sys.version_info.minor}',
        #     'site-packages', package_name, 'ego_state_package',
        #     'params'), glob(os.path.join('params', '*.csv'))),
        (os.path.join(
            'lib', f'python{sys.version_info.major}.{sys.version_info.minor}',
            'site-packages', package_name, 'ego_state_cv', 'params'), glob('params/*.yaml')),
        # (os.path.join(
        # 'lib',
        # f'python{sys.version_info.major}.{sys.version_info.minor}',
        # 'site-packages', package_name, 'ego_state_package'),
        # glob(os.path.join('fusion', 'ego_state_package', '*.py')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='wabco',
    maintainer_email='you@example.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    include_package_data=True,
    entry_points={
        'console_scripts': ['node_fusion = fusion.node_fusion:main'],
    },
)
