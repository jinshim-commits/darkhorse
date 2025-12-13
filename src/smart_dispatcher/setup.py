from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'smart_dispatcher'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),   # <-- 반드시 이렇게 해야 함
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # launch 파일 설치
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='flynn',
    maintainer_email='flynn@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dispatcher_node = smart_dispatcher.dispatcher_node:main',
        ],
    },
)
