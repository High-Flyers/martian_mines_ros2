<<<<<<< HEAD
from setuptools import setup
import os
from glob import glob

package_name = "martian_mines_ros2"

# Check if requirements.txt exists
requirements_path = os.path.join(os.path.dirname(__file__), "requirements.txt")
if os.path.exists(requirements_path):
    with open(requirements_path) as f:
        requirements = f.read().splitlines()
else:
    requirements = []

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    install_requires=requirements,
    extras_require={"dev": ["pytest"]},
    zip_safe=True,
    maintainer="highflyers",
    maintainer_email="highflyers.polsl@gmail.com",
    description="Martian Mines ROS2 package",
    entry_points={
        "console_scripts": [
            "figure_finder = martian_mines_ros2.figure_finder:main",
            "precision_landing = martian_mines_ros2.precision_landing:main",
            "bbox_publisher = martian_mines_ros2.bbox_publisher:main",
            "detection = martian_mines_ros2.detection:main",
            "trajectory_generator = martian_mines_ros2.trajectory_generator:main",
            "report_uploader = martian_mines_ros2.utils.report_uploader:main",
            "environment_visualization = martian_mines_ros2.environment_visualization:main",
            "trajectory_tracker = martian_mines_ros2.trajectory_tracker:main",
            "detections_visualization = martian_mines_ros2.detections_visualization:main",
            # "mission_controller = martian_mines_ros2.mission_controller:main",
        ],
    },
    data_files=[
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
    ],
)
=======
from distutils.core import setup
from setuptools import find_packages
from catkin_pkg.python_setup import generate_distutils_setup

with open('requirements.txt') as f:
    requirements = f.read().splitlines()

setup_args = generate_distutils_setup(
    packages=find_packages('src'),
    package_dir={'': 'src'},
    test_suite='tests',
    install_requires=requirements,
    extras_resuire={
        'dev': [
            'pytest'
        ]
    },
)

setup(**setup_args)
>>>>>>> mission_control
