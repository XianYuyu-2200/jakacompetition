from setuptools import setup
from glob import glob
import os

package_name = "jaka_competition_kit"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resources/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "config"), glob("config/*")),
        (os.path.join("share", package_name, "launch"), glob("launch/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Competition Organizer",
    maintainer_email="organizer@example.com",
    description="JAKA Mini 2 比赛工具包",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "scene_generator = jaka_competition_kit.scene_generator:main",
            "scorer = jaka_competition_kit.scorer:main",
            "ref_track1 = jaka_competition_kit.ref_track1:main",
            "ref_track2 = jaka_competition_kit.ref_track2:main",
            "arena_check = jaka_competition_kit.arena_check:main",
            "qr_tool = jaka_competition_kit.qr_tool:main",
        ],
    },
)
