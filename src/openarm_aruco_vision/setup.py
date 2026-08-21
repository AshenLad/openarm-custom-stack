from glob import glob

from setuptools import find_packages, setup


package_name = "openarm_aruco_vision"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/worlds", glob("worlds/*.sdf")),
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="Asukaandmeaaa",
    maintainer_email="asukaandmeaaa@users.noreply.github.com",
    description="OpenArm ArUco calibration and RGB-D scene perception",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "aruco_detector = openarm_aruco_vision.aruco_detector_node:main",
            "blue_cube_detector = openarm_aruco_vision.blue_cube_detector_node:main",
            "box_detector = openarm_aruco_vision.box_detector_node:main",
        ],
    },
)
