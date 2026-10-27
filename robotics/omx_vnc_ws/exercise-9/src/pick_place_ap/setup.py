from setuptools import find_packages, setup

package_name = 'pick_place_ap'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'pick_place_ap_interface'],
    zip_safe=True,
    maintainer='Gangotri',
    maintainer_email='kasturigangotri72@gmail.com',
    description='Action Preemption',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'pick_place_ap_server = pick_place_ap.pick_place_ap_server:main',
            'pick_place_ap_client = pick_place_ap.pick_place_ap_client:main',
        ],
    },
)
