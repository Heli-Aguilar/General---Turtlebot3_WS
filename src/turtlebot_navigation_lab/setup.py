from setuptools import find_packages, setup

package_name = 'turtlebot_navigation_lab'

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
    maintainer='zerotwo',
    maintainer_email='heli.aguilar.5407@gmial.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        	'state_estimator = homework.state_estimator:main',
        	'controller = homework.controller:main',
        	'forward_simulator = homework.forward_simulator:main',
        	'planner = homework.planner:main',
        ],
    },
)
