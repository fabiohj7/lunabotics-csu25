from setuptools import find_packages, setup

package_name = 'ctrl_to_motor'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/startup.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lunabotics',
    maintainer_email='lunabotics@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ctrl_to_motor = ctrl_to_motor.ctrl_to_motor:main'
        ],
    },
)
