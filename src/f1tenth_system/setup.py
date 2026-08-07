from setuptools import find_packages, setup
import os
from glob import glob
import shutil
import stat
import sys

from setuptools.command.develop import develop as setuptools_develop


class ColconDevelopCompat(setuptools_develop):
    """Compatibility shim for colcon + setuptools >= 80.

    ROS 2 Humble's colcon Python build task still calls legacy setuptools
    develop options such as ``--uninstall``, ``--editable`` and
    ``--build-directory``. New setuptools versions removed those options from
    ``develop``. This command accepts the legacy options and performs the small
    amount of work colcon needs for symlink installs: expose scripts from
    ``lib/<package>`` while colcon supplies PYTHONPATH and data-file symlinks.
    """

    user_options = list(setuptools_develop.user_options) + [
        ('uninstall', None, 'accept legacy colcon develop uninstall mode'),
        ('editable', None, 'accept legacy colcon editable develop mode'),
        ('build-directory=', None, 'accept legacy colcon build directory'),
        ('script-dir=', None, 'install scripts to DIR'),
    ]
    boolean_options = list(setuptools_develop.boolean_options) + [
        'uninstall',
        'editable',
    ]

    def initialize_options(self):
        self.install_dir = None
        self.no_deps = False
        self.user = False
        self.prefix = None
        self.index_url = None
        self.uninstall = False
        self.editable = False
        self.build_directory = None
        self.script_dir = None

    def finalize_options(self):
        # setuptools >= 80 intentionally keeps this command minimal. Avoid
        # invoking the pip-backed implementation and compute only script_dir.
        if self.script_dir is None:
            install_cmd = self.get_finalized_command('install')
            self.script_dir = install_cmd.install_scripts
        self.script_dir = self._expand_colcon_path(self.script_dir)

    def run(self):
        if self.uninstall:
            self._uninstall_scripts()
            return
        self._install_scripts()

    def _expand_colcon_path(self, path):
        if path is None:
            return None
        return path.replace('$base', sys.prefix).replace('$platbase', sys.exec_prefix)

    def _install_scripts(self):
        scripts = self.distribution.scripts or []
        if not scripts:
            return

        os.makedirs(self.script_dir, exist_ok=True)
        for script in scripts:
            src = os.path.realpath(script)
            dst = os.path.join(self.script_dir, os.path.basename(script))

            if os.path.lexists(dst):
                os.remove(dst)

            if self.editable:
                os.symlink(src, dst)
            else:
                shutil.copy2(src, dst)

            mode = os.stat(src).st_mode
            os.chmod(dst, mode | stat.S_IXUSR | stat.S_IXGRP | stat.S_IXOTH)

    def _uninstall_scripts(self):
        scripts = self.distribution.scripts or []
        for script in scripts:
            dst = os.path.join(self.script_dir, os.path.basename(script))
            if os.path.lexists(dst):
                os.remove(dst)

package_name = "f1tenth_system"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    scripts=glob(os.path.join("scripts", "*.py")),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "launch"),
            glob(os.path.join("launch", "*launch.[pxy][yma]*")),
        ),
        (
            os.path.join("share", package_name, "params"),
            glob(os.path.join("params", "*.yaml")),
        ),  # 包含 yaml 文件
        (
            os.path.join("share", package_name, "params"),
            glob(os.path.join("params", "*.json")),
        ),  # 包含 yaml 文件
        (
            os.path.join("share", package_name, "behaviour_trees"),
            glob(os.path.join("behaviour_trees", "*.xml")),
        ),
        (
            os.path.join("share", package_name, "rviz"),
            glob(os.path.join("rviz", "*.rviz")),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="EleSheep",
    maintainer_email="ZhizhaoZhang@gmail.com",
    description="TODO: Package description",
    license="MIT",
    extras_require={
        "test": ["pytest"],
    },
    cmdclass={
        "develop": ColconDevelopCompat,
    },
)
