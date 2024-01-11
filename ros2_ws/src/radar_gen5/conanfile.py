import fileinput
import os
import re
import subprocess
from conans import ConanFile, tools

class RadarGen5Conan(ConanFile):
    name = "radar_gen5"
    version = "1.0.7"
    settings = "os", "arch", "compiler", "build_type"

    requires = (
        "diagnostic_updater/1.9.3-0-maploc-3@maploc/stable",
        "geometry_msgs/1.12.7-0-maploc-3@maploc/stable",
        "message_generation/0.4.0-0-maploc-3@maploc/stable",
        "pcl_ros/1.4.4-0-maploc-3@maploc/stable",
        "std_msgs/0.5.11-0-maploc-3@maploc/stable",
        "tf/1.11.9-0-maploc-3@maploc/stable",
        "tf2/0.5.20-0-maploc-3@maploc/stable",
        "tf2_geometry_msgs/0.5.20-0-maploc-3@maploc/stable",
        "vfc/1903.4.0-maploc-2@maploc/stable",
        "yaml-cpp/0.6.3@maploc/stable",
    )

    build_requires = (
        "catkin/0.7.20-1-maploc-3@maploc/stable",
        "cmake/3.18.1-maploc-2@maploc/stable",
    )

    generators = "cmake_paths", "virtualenv"

    _source_subfolder = "src"

    def export_sources(self):
        self.copy("*", dst=self._source_subfolder)

    def system_requirements(self):
        sysdeps = (
            'python-catkin-tools',
        )
        apttool = tools.AptTool()
        installer = tools.SystemPackageTool(tool=apttool)
        anything_to_install = False
        for dep in sysdeps:
            if not apttool.installed(dep):
                anything_to_install = True
        if anything_to_install:
            installer.add_repository(
                "http://packages.ros.org/ros/ubuntu",
                repo_key="'http://keyserver.ubuntu.com/pks/lookup?op=get&search=0xC1CF6E31E6BADE8868B172B4F42ED6FBAB17C654'")
        for dep in sysdeps:
            installer.install(dep)

    def build(self):
        conan_paths_file = os.path.join(self.install_folder, "conan_paths.cmake")
        source_folder = os.path.join(self.source_folder, self._source_subfolder)
        if self.should_configure:
            config_command = (
                "catkin", "config",
                "-j{}".format(tools.cpu_count()),
                "--source-space", source_folder,
                "--install-space", self.package_folder,
                "--install",
                "--merge-install",
                "--cmake-args",
                "-DCMAKE_PREFIX_PATH={}".format(self.package_folder),
                "-DCMAKE_TOOLCHAIN_FILE={}".format(conan_paths_file),
                "-DCMAKE_BUILD_TYPE={}".format(self.settings.build_type),
                "--",
            )
            subprocess.check_call(config_command)
        if self.should_build:
            subprocess.check_call(("catkin", "build"))
        if self.should_test:
            subprocess.check_call(("catkin", "run_tests"))

    def package(self):
        for config in self._find_cmake_configs():
            self._remove_absolute_paths(config)

    def package_info(self):
        self.env_info.PYTHONPATH.append(os.path.join(self.package_folder, "lib", "python2.7", "dist-packages"))

    def _find_cmake_configs(self):
        for dir, _, files in os.walk(self.package_folder):
            for file in files:
                if re.search(r'^[-_.a-zA-Z0-9]+config\.cmake$', file, re.IGNORECASE) is not None:
                    yield os.path.join(dir, file)

    def _remove_absolute_paths(self, cmake_config):
        for line in fileinput.input(cmake_config, inplace=True):
            line = line.rstrip()
            line = re.sub(r'(/[^/;"(): ]+)+/.conan/data', r'$ENV{HOME}/.conan/data', line)
            print(line)
