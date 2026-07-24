from setuptools import find_packages, setup

package_name = "weedwatch_coordinator"

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
    description="관통 코디네이터 노드 (두둑 순회·타격·로깅)",
    license="MIT",
    entry_points={
        "console_scripts": [
            "coordinator_node = weedwatch_coordinator.coordinator_node:main",
        ],
    },
)
