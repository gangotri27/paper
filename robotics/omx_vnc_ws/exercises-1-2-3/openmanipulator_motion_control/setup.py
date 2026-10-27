from setuptools import find_packages, setup

package_name = 'openmanipulator_motion_control'

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
    maintainer='Gangotri',
    maintainer_email='kasturigangotri72@gmail.com',
    description='Motion Control',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'simul_control = openmanipulator_motion_control.simul_control:main',
            'sequ_control = openmanipulator_motion_control.sequ_control:main',
            'intera_control = openmanipulator_motion_control.intera_control:main',
            'sync_control = openmanipulator_motion_control.sync_controller:main',
        ],
    },
)
