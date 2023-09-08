#!/usr/bin/env python

from setuptools import setup

package_name = 'hazard_status_monitor'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    package_dir={'': 'src'},
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['plugin.xml']),
        ('share/' + package_name + '/resource',
         ['resource/hazard_status_monitor_widget.ui'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author="Aaron Blasdel",
    maintainer="Aaron Blasdel",
    maintainer_email="ablasdel@gmail.com",
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description=(
        'hazard_status_monitor provides a GUI plugin viewing '
        'DiagnosticsArray messages.'
    ),
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hazard_status_monitor = hazard_status_monitor.hazard_status_monitor:main'
        ]
    }
)
