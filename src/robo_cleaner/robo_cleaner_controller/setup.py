from setuptools import setup

package_name = 'robo_cleaner_controller'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tisho',
    maintainer_email='tisho@taxime.to',
    description='robo_cleaner_controller',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robo_cleaner_controller = robo_cleaner_controller.robo_cleaner_controller:main'
        ],
    },
)
