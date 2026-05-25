import os
from glob import glob

from setuptools import find_packages
from setuptools import setup


package_name = "drive_manager"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "map"), glob("map/*")),
        (os.path.join("share", package_name, "param"), glob("param/*.yaml")),
        (os.path.join("share", package_name, "rviz"), glob("rviz/*.rviz")),
        (os.path.join("share", package_name, "urdf"), glob("urdf/*.urdf")),
        (
            os.path.join("share", package_name, "meshes", "bases"),
            glob("meshes/bases/*.stl"),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="root",
    maintainer_email="root@example.com",
    description="Drive mission manager nodes for controlling the robot over ROS 2.",
    license="Apache License 2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "command_manager = drive_manager.command_manager:main",
            "mission_driver = drive_manager.mission_driver:main",
            "two_point = drive_manager.two_point:main",
        ],
    },
)
