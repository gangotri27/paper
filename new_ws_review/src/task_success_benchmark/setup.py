from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'task_success_benchmark'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Gangotri',
    maintainer_email='kasturigangotri72@gmail.com',
    description='Task Success Benchmark for 5-DOF vs 6-DOF comparison',
    license='Apache 2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'run_task_benchmark = task_success_benchmark.scripts.run_task_benchmark:main',
            'analyze_results = task_success_benchmark.scripts.analyze_results:main',
        ],
    },
)
