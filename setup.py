import setuptools

setuptools.setup(
    name="rplidar",
    version="0.0.2",
    author="Ben Lewis",
    author_email="ben.lws12@gmail.com",
    description="Tools to use and rplidar",
    long_description="Designed and tested for the Rplidar A1",
    package_dir={'': 'src'},
    packages=setuptools.find_packages(),
    python_requires='>=3.6',
    install_requires=['bitstring', 'pyserial']
)