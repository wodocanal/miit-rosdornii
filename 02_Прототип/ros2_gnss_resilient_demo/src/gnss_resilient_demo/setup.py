from glob import glob
from setuptools import find_packages, setup

package_name = "gnss_resilient_demo"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*.json")),
    ],
    install_requires=["setuptools", "numpy"],
    zip_safe=True,
    maintainer="Hackathon Team",
    maintainer_email="team@example.com",
    description="GNSS-resilient transport positioning demonstration",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "demo_gui = gnss_resilient_demo.gui:main",
            "demo_headless = gnss_resilient_demo.headless:main",
            "frob_adapter = gnss_resilient_demo.ros_node:main",
        ],
    },
)
