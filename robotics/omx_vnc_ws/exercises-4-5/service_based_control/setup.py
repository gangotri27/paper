from setuptools import find_packages, setup

package_name = 'service_based_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'service_based_control_interface'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='kasturigangotri72@gmail.com',
    description='Service Arm Movement',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'move_arm_server = service_based_control.move_arm_server:main',
            'move_arm_client = service_based_control.move_arm_client:main',
            'set_pose_server = service_based_control.set_pose_server:main',
            'set_pose_client = service_based_control.set_pose_client:main',
        ],
    },
)
