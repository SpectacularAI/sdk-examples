from setuptools import setup

package_name = 'spectacularai_depthai'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Spectacular AI',
    maintainer_email='firstname.lastname@spectacularai.com',
    description='Spectacular AI plugin for DepthAI',
    license='All Rights Reserved',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ros2_node = spectacularai_depthai.ros2_node:main'
        ],
    },
)
