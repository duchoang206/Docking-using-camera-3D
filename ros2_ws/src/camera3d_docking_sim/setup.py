import os
from glob import glob

from setuptools import find_packages, setup

package_name = "camera3d_docking_sim"


def share(sub, pattern):
    return (os.path.join("share", package_name, sub), glob(os.path.join(sub, pattern)))


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        share("launch", "*.launch.py"),
        share("config", "*.yaml"),
        share("worlds", "*.sdf"),
        share("models/docking_bot", "model.*"),
        share("scripts", "*.sh"),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Docking3D Dev",
    maintainer_email="todo@example.com",
    description="Mô phỏng Gazebo cho camera3d_docking",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "generate_world = camera3d_docking_sim.generate_world:main",
            "e2e_docking_test = camera3d_docking_sim.e2e_docking_test:main",
        ],
    },
)
