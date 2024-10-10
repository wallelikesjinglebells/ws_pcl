from setuptools import setup

import os

package_name = 'octomap_publisher'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
            # For getting rid of this error: doesn't explicitly install a marker in the package index
            # Solution was found here https://answers.ros.org/question/367328/ament_python-package-doesnt-explicitly-install-a-marker-in-the-package-index/
        (os.path.join('share', package_name), ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jing',
    maintainer_email='jing.wang@tum.de',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'octomap_publisher_node = octomap_publisher.octomap_publisher_node:main',
        ],
    },
)
