import os
from glob import glob
from setuptools import find_packages, setup

package_name = "solar_mower_sim"


def tree(src):
    """ディレクトリ配下を再帰的に data_files 用に列挙"""
    out = []
    for root, _, files in os.walk(src):
        if files:
            out.append((os.path.join("share", package_name, root),
                        [os.path.join(root, f) for f in files]))
    return out


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
        (os.path.join("share", package_name, "config"), glob("config/*")),
        (os.path.join("share", package_name, "worlds"), glob("worlds/*")),
        (os.path.join("share", package_name, "prompts"), glob("prompts/*")),
    ] + tree("models"),
    install_requires=["setuptools", "numpy", "requests"],
    zip_safe=True,
    maintainer="PAI final project",
    maintainer_email="pai@example.com",
    description="Tracked solar-farm mowing robot simulation with LLM commands.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "mission_executor = solar_mower_sim.mission_executor:main",
            "llm_commander = solar_mower_sim.llm_commander:main",
        ],
    },
)
