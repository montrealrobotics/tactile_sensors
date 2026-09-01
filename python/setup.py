from setuptools import setup, find_packages

setup(
    name="robotiq_tactile_sensor",
    version="0.1.0",
    packages=find_packages(),
    py_modules=["protocol"],
    package_data={"": ["web/*"]},
    include_package_data=True,
)
