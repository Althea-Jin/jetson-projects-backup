from glob import glob
from setuptools import find_packages, setup

package_name = "scan_cmd_vel_follower"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/config", glob("config/*.yaml")),
        (f"share/{package_name}/launch", glob("launch/*.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="amov",
    maintainer_email="amov@example.com",
    description="Conservative path follower for SCAN-Planner Twist output.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "scan_cmd_vel_follower = scan_cmd_vel_follower.follower_node:main",
        ],
    },
)
