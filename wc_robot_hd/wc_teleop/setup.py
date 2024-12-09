from setuptools import find_packages
from setuptools import setup

package_name = 'wc_teleop'

setup(
    name=package_name,
    version='2.1.5',
    packages=find_packages(exclude=[]),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
    ],
    zip_safe=True,
    maintainer='vikas_maurya',
    maintainer_email='22ee01049@iitbbs.ac.in',
    description='TODO: Package description',
    license='TODO: License declaration',
    keywords=['ROS'],
    description=(
        'Teleoperation node using keyboard for wc.'
    ),
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'teleop_keyboard = wc_teleop.script.teleop_keyboard:main'
        ],
    },
)