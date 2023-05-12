from setuptools import setup
import os
from glob import glob

package_name = "sfr"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name), glob("launch/*_launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="sfr",
    maintainer_email="hbel@itu.dk, apal@itu.dk, erav@itu.dk",
    description="Robot for indoor detection of plant diseases.",
    license="N/A",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "encoder_publisher_node = sfr.encoder_publisher_node:main",
            "ultrasonic_publisher_node = sfr.ultrasonic_publisher_node:main",
            "motor_subscriber_node = sfr.motor_subscriber_node:main",
            "crop_follower_node = sfr.crop_follower_node:main",
            "scd30_publisher_node = sfr.scd30_publisher_node:main",
        ],
    },
)
