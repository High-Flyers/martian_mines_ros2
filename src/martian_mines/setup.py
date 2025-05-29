#include information how to install the package

from setuptools import find_packages, setup
from glob import glob

package_name = 'martian_mines'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ("share/" + package_name + "/launch", glob("launch/*.launch.xml")),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/config", glob("config/*.json")),
        ("share/" + package_name + "/nn_models", glob("nn_models/*.pt")),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='highflyers',
    maintainer_email='highflyers.polsl0@gmail.com',
    description='Martian mines system',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "figure_finder = martian_mines.src.figure_finder:main",
            "precision_landing = martian_mines.src.precision_landing:main",
            "bbox_publisher = martian_mines.src.bbox_publisher:main",
            "detection = martian_mines.src.detection:main",
            "trajectory_generator = martian_mines.src.trajectory_generator:main",
            "report_uploader = martian_mines.src.utils.report_uploader:main",
            "environment_visualization = martian_mines.src.environment_visualization:main",
            "trajectory_tracker = martian_mines.src.trajectory_tracker:main",
            "detection_visualization = martian_mines.src.detection_visualization:main",
            "mission_controller = martian_mines.src.mission_controller:main",
            "tf_start_pose = martian_mines.src.tf_start_pose:main",
            "data_collector = martian_mines.src.data_collector:main",
            "camera_recorder = martian_mines.src.camera_recorder:main",
            "camera_remap = martian_mines.src.camera_remap:main",
            ],
    },
)
