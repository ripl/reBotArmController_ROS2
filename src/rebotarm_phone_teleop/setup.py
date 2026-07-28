from glob import glob

from setuptools import find_packages, setup


package_name = "rebotarm_phone_teleop"

setup(
    name=package_name,
    version="0.3.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    test_suite="test",
    zip_safe=True,
    maintainer="reBotArm Maintainers",
    maintainer_email="support@example.com",
    description="Position-only phone teleoperation for reBotArm.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "phone_eef_teleop = rebotarm_phone_teleop.teleop_node:main",
        ],
    },
)
