from setuptools import setup

package_name = 'ur_controller'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'numpy', 'autologging'],
    zip_safe=True,
    maintainer='tisho',
    maintainer_email='tisho@taxime.to',
    description='Ocado Robotics Accelerator 2022 Pick & Place Task',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ur_controller = ur_controller.ur_controller:main'
        ],
    },
)
