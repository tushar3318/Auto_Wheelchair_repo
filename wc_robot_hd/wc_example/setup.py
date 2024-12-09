from setuptools import find_packages, setup

package_name = 'wc_example'

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
    maintainer='vikas_maurya',
    maintainer_email='22ee01049@iitbbs.ac.in',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'wc_obstacle_detection = \
                wc_example.wc_obstacle_detection.main:main',
            'wc_patrol_client = \
                wc_example.wc_patrol_client.main:main',
            'wc_patrol_server = \
                wc_example.wc_patrol_server.main:main',
            'wc_position_control = \
                wc_example.wc_position_control.main:main',
        ],
    },
)
