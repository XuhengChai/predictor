from setuptools import setup, find_packages
import os
import sys
from glob import glob

package_name = 'zf_traj_prediction'

setup(
    name=package_name,
    version='0.0.0',
    # packages=[package_name],
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join(
            'lib', f'python{sys.version_info.major}.{sys.version_info.minor}',
            'site-packages', package_name),
             glob(os.path.join('zf_traj_prediction', '*.py'))),
        # install resource files if any
        (os.path.join('share', package_name, 'model'), glob('model/*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='xh',
    maintainer_email=' xuheng.chai@zf.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'traj_prediction = zf_traj_prediction.traj_prediction:main'
        ],
    },
)
