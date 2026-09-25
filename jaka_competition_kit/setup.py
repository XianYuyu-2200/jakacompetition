from setuptools import setup
from glob import glob
import os

package_name = "jaka_competition_kit"


def _data(pattern):
    """挑 data_files 用的文件, 跳过运行时生成的 ``__pycache__``。

    launch 目录里的 .py 一旦被跑过, Python 就会在旁边生成 ``launch/__pycache__``。
    裸 ``glob("launch/*")`` 会把它当数据一起收进去, 而 colcon --symlink-install
    随后会试着把"目录"当文件链接到 install, 直接报
    ``can't copy ... launch/__pycache__: doesn't exist or not a regular file``,
    整个包编译失败 —— 只在"跑过仿真之后再 build"时复现, 很容易踩。
    """
    return [p for p in glob(pattern) if os.path.isfile(p)]


setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resources/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "config"), _data("config/*")),
        (os.path.join("share", package_name, "launch"), _data("launch/*")),
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
            "gz_scene = jaka_competition_kit.gz_scene:main",
        ],
    },
)
