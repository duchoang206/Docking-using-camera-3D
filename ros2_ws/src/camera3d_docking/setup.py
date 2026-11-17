import os
from glob import glob

from setuptools import find_packages, setup

package_name = "camera3d_docking"


def share(sub, pattern):
    return (os.path.join("share", package_name, sub), glob(os.path.join(sub, pattern)))


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        share("config", "*.yaml"),
        share("launch", "*.launch.py"),
        share("templates", "*.npz"),
        share("rviz", "*.rviz"),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Docking3D Dev",
    maintainer_email="todo@example.com",
    description="Docking robot bằng camera 3D (PointCloud2) cho phần cứng yếu",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "pose_estimator = camera3d_docking.nodes.pose_estimator_node:main",
            "docking_controller = camera3d_docking.nodes.docking_controller_node:main",
            "cloud_saver = camera3d_docking.nodes.cloud_saver_node:main",
            "make_template = camera3d_docking.tools.make_template:main",
            "benchmark = camera3d_docking.tools.benchmark:main",
        ],
    },
)
