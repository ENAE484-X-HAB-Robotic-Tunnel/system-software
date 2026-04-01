from setuptools import find_packages, setup

package_name = "tunnel_ik"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ENAE484 X-HAB Robotic Tunnel",
    description="tunnel_ik for the X-HAB Robotic Tunnel system.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "controller_node = tunnel_ik.ik_node:main",
        ],
    },
)
