import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'rob599_hw2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name), glob('srv/*.srv')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jorge',
    maintainer_email='jlruballos@gmail.com',
    description='ROB599 HW2',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'PublishTwist = rob599_hw2.PublishTwist:main',
            'TwistChecker = rob599_hw2.TwistChecker:main',
            'VelocityLimiter = rob599_hw2.VelocityLimiter:main',
            'nasa_action_server = rob599_hw2.nasa_action_server:main',
            'nasa_action_client = rob599_hw2.nasa_action_client:main',
        ],
    },
)
