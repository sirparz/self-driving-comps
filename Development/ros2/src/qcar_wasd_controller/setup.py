from setuptools import setup

package_name = 'qcar_wasd_controller'

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
    maintainer='fs',
    maintainer_email='as1605258@qu.edu.qa',
    description='A simple WASD keyboard teleop node for QCar',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'wasd_teleop = qcar_wasd_controller.wasd_teleop:main',
        ],
    },
)