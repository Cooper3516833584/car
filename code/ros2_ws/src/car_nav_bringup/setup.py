from glob import glob
from setuptools import find_packages, setup

setup(name="car_nav_bringup", version="0.1.0", packages=find_packages(), install_requires=["setuptools"], zip_safe=True, data_files=[("share/ament_index/resource_index/packages", ["resource/car_nav_bringup"]), ("share/car_nav_bringup", ["package.xml"]), ("share/car_nav_bringup/launch", glob("launch/*.launch.py")), ("share/car_nav_bringup/config", glob("config/*")), ("share/car_nav_bringup/behavior_trees", glob("behavior_trees/*")), ("share/car_nav_bringup/urdf", glob("urdf/*")), ("share/car_nav_bringup/maps", glob("maps/*"))], entry_points={"console_scripts": ["generate_field_map = car_nav_bringup.generate_field_map:main"]})
