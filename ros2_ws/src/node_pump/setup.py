import os
from glob import glob
from setuptools import find_packages, setup

package_name = "node_pump"

setup(
    name = package_name,
    version = "0.0.1",
    packages = find_packages(exclude = ["test"]),
    data_files = [
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        (os.path.join("share", package_name), glob("package.xml")),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires = ["setuptools"],
    zip_safe = True,
    maintainer = "Paz",
    maintainer_email = "pasetto.niccolo@studenti.unipd.it",
    description = "Node to controll the pumps.",
    license = "Apache-2.0",
    entry_points = {
        "console_scripts": [
            "node_pump = " + package_name + ".node_pump:main",
        ],
    },
)
