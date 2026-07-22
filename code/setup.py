from setuptools import find_packages, setup

package_name = "car_components"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=("test", "test.*")),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="car maintainers",
    maintainer_email="maintainer@example.invalid",
    description="Hardware-independent car component package",
    license="Proprietary",
)
