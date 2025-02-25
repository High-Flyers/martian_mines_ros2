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
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "precision_landing = martian_mines_ros2.precision_landing:main",
            "offboard = martian_mines_ros2.drone.offboard:main",
            "report_uploader = martian_mines_ros2.utils.report_uploader:main",
        ],
    },
    data_files=[
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
    ],
)
