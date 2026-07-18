from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'aal'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ega',
    maintainer_email='EGAlberts@github.com',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'adaptation_layer = aal.adaptation_manager_node:main',
            'demo_publisherA = aal.demos.publisher_A:main',
            'demo_publisherB = aal.demos.publisher_B:main',
            'demo_subscriber = aal.demos.subscriber:main',
            'demo_swap_publishers = aal.demos.swap_two_publishers:main',
            'demo_change_topics = aal.demos.change_topics:main',
            'test_adap_strategy = aal.testing.max_velocity:main',
        ],
    },
)
