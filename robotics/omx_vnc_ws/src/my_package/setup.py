from setuptools import find_packages, setup

package_name = 'my_package'

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
    description='Arm Movement',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'my_node = my_package.my_node:main',
            'left = my_package.left:main',
            'right = my_package.right:main',
            'up = my_package.up:main',
            'down = my_package.down:main',
        ],
    },
)
