from setuptools import setup

setup(
    name="UR12ID",
    version="0.1.0",
    description="Python library to control an UR robot with Robotiq Gripper and Wrist Camera",
    author="Byeongdu Lee",
    author_email="blee-at-anl.gov",
    url='https://github.com/Byeongdulee/UR_12idb',
    packages=["UR12ID"],
    provides=["UR12ID"],
    install_requires=["urx", "numpy", "math3d", "pyzbar", "pupil-apriltags"],
    license="GNU Lesser General Public License v3",
    classifiers=[
        "Programming Language :: Python",
        "Programming Language :: Python :: 3",
        "Development Status :: 1 - Beta",
        "Intended Audience :: Developers",
        "Operating System :: OS Independent",
        "Topic :: System :: Hardware :: Hardware Drivers",
        "Topic :: Software Development :: Libraries :: Python Modules",
    ])