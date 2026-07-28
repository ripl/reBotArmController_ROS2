from glob import glob

from setuptools import find_packages, setup


package_name = "rebotarm_phone_bridge"

setup(
    name=package_name,
    version="0.3.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools", "numpy"],
    test_suite="test",
    zip_safe=True,
    maintainer="reBotArm Maintainers",
    maintainer_email="support@example.com",
    description="ROS 2 bridge for the reBotArm iPhone pose stream.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "phone_tracking_bridge = rebotarm_phone_bridge.bridge_node:main",
            "phone_calibration_keyboard = "
            "rebotarm_phone_bridge.calibration_keyboard:main",
        ],
    },
)
