from setuptools import find_packages, setup


package_name = "x1_coffee_device_sdk_py"

setup(
    name=package_name,
    version="0.3.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="X1 SDK Maintainers",
    maintainer_email="sdk-maintainers@example.com",
    description="Python device clients for the X1 Coffee ROS 2 SDK.",
    license="Proprietary",
)
