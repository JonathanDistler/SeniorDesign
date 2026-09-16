from setuptools import find_packages, setup

package_name = "swarm_mavsdk_ros2"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            [f"resource/{package_name}"],
        ),
        (
            f"share/{package_name}",
            ["package.xml"],
        ),
    ],
    install_requires=[
        "setuptools",
    ],
    zip_safe=True,
    description="ROS 2 / PX4 multi-drone swarm controller.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "swarm_tests = swarm_mavsdk_ros2.swarm_node:main",
        ],
    },
)