from setuptools import find_packages, setup

package_name = 'limo_speed_ctrl'

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
    maintainer='flynn',
    maintainer_email='youdongoh67@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            # 1. 명령을 내리는 웹 리모컨
            'web_remote = limo_speed_ctrl.web_remote:main',
            # 2. 명령을 받아 속도를 조절하는 매니저
            'nav2_speed_manager = limo_speed_ctrl.nav2_speed_manager:main',
            'keyboard_remote = limo_speed_ctrl.keyboard_remote:main',
        ],
    },
)
