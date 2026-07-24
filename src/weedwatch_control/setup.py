from setuptools import find_packages, setup

package_name = "weedwatch_control"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="JunH95",
    maintainer_email="wol00107@gmail.com",
    description="주행+관절 제어 노드 (ww_cmd 대체, rclpy)",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "control_node = weedwatch_control.control_node:main",
        ],
    },
)
