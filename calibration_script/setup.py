from setuptools import setup

package_name = 'calibration_script'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='prakt5',
    maintainer_email='prakt5@todo.todo',
    description='Publishes calibration data and subcribes to tracking system and robot data',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        	'calibration = calibration_script.calibration:main'
        ],
    },
)
