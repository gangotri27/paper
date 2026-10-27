from setuptools import find_packages, setup

package_name = 'py_srvcli'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'srvcli'],
    zip_safe=True,
    maintainer='Gangotri',
    maintainer_email='kasturigangotri72@gmail.com',
    description='Service Movement',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'direction_service_server = py_srvcli.direction_service_server:main',
            'direction_service_client = py_srvcli.direction_service_client:main',
            'setpose_service_server = py_srvcli.setpose_service_server:main',
            'setpose_service_client = py_srvcli.setpose_service_client:main',
            'performtask_service_server = py_srvcli.performtask_service_server:main',
            'performtask_service_client = py_srvcli.performtask_service_client:main',
        ],
    },
)
