from setuptools import find_packages, setup

package_name = "arm_demos"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="Abhinav Garg",
    maintainer_email="abhinavgrg815@gmail.com",
    description="Motion demos for the 6-DOF arm.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "move_to = arm_demos.move_to:main",
            "cartesian_to = arm_demos.cartesian_to:main",
        ],
    },
)
