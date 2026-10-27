from setuptools import setup
import os
from glob import glob

package_name = 'omx_benchmark'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml') if os.path.exists('config') else []),
    ],
    install_requires=[
        'setuptools',
        'numpy',
        'scipy',
        'matplotlib',
        'pandas',
        'seaborn',
    ],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='ubuntu@localhost',
    description='Benchmark for 5-DOF vs 6-DOF OpenManipulator-X',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'scene_builder = omx_benchmark.scene_builder:main',
            'benchmark_runner = omx_benchmark.benchmark_runner:main',
            'generate_goals = omx_benchmark.generate_goals:main',
            'task_goal_generator = omx_benchmark.task_goal_generator:main',
            'task_goal_generator_simple = omx_benchmark.task_goal_generator_simple:main',
            'planning_benchmark_runner = omx_benchmark.planning_benchmark_runner:main',
            'planning_analyzer = omx_benchmark.planning_analyzer:main',
            'statistical_analyzer = omx_benchmark.statistical_analyzer:main',
            'csv_exporter = omx_benchmark.csv_exporter:main',
            'latex_exporter = omx_benchmark.latex_exporter:main',
            'warehouse_manager = omx_benchmark.warehouse_manager:main',
            'workspace_explorer = omx_benchmark.workspace_explorer:main',
            'workspace_explorer_advanced = omx_benchmark.workspace_explorer_advanced:main',
            'final_goal_generator = omx_benchmark.final_goal_generator:main',
            'pose_goal_generator = omx_benchmark.pose_goal_generator:main',
            'single_robot_goal_generator = omx_benchmark.single_robot_goal_generator:main',
            'scene_builder_variable = omx_benchmark.scene_builder_variable:main',
            'scene_builder_fixed = omx_benchmark.scene_builder_fixed:main',
            'generate_5dof_goals = omx_benchmark.generate_5dof_goals:main',
        ],
    },
)