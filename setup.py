"""
setup for the arlib project
Copyright (C) 2018 Leonardo Arcari (leonardo1.arcari@gmail.com)
License: MIT License   See LICENSE.md for the full license.

This file basically just uses CMake to compile the dlib python bindings project
located in the tools/python folder and then puts the outputs into standard
python packages.
To build the dlib:
    python setup.py build
To build and install:
    python setup.py install
To package the wheel (after pip installing twine and wheel):
    python setup.py bdist_wheel
To upload the binary wheel to PyPi
    twine upload dist/*.whl
To upload the source distribution to PyPi
    python setup.py sdist 
    twine upload dist/dlib-*.tar.gz
To exclude/include certain options in the cmake config use --yes and --no:
    for example:
    --yes USE_AVX_INSTRUCTIONS: will set -DUSE_AVX_INSTRUCTIONS=yes
    --no USE_AVX_INSTRUCTIONS: will set -DUSE_AVX_INSTRUCTIONS=no
Additional options:
    --compiler-flags: pass flags onto the compiler, e.g. --compiler-flags "-Os -Wall" passes -Os -Wall onto GCC.
    --exe-linker-flags: pass flag onto the linker for executable targets, e.g. "-L/usr/local/opt/llvm/lib -Wl,-rpath,/usr/local/opt/llvm/lib"
    --module-linker-flags: pass flag onto the linker for module targets, e.g. "-L/usr/local/opt/llvm/lib -Wl,-rpath,/usr/local/opt/llvm/lib"
    -G: Set the CMake generator.  E.g. -G "Visual Studio 14 2015"
    --clean: delete any previous build folders and rebuild.  You should do this if you change any build options
             by setting --compiler-flags or --yes or --no since last time you ran a build to make sure the changes
             take effect.
    --set: set arbitrary options e.g. --set CUDA_HOST_COMPILER=/usr/bin/gcc-6.4.0

Many thanks to Ehsan Azar (dashesy@linux.com) for the script:
https://github.com/davisking/dlib/blob/master/setup.py
"""

from setuptools import setup, Extension
from setuptools.command.build_ext import build_ext
from distutils.version import LooseVersion

import re
import os
import errno
import platform
import stat
import shutil
import subprocess
import sys
from distutils import log


def get_extra_cmake_options():
    """read --clean, --yes, --no, --set, --compiler-flags, --exe-linker-flags,
    --module-linker-flags, and -G options from the command line and add them as
    cmake switches.
    """
    _cmake_extra_options = []
    _clean_build_folder = False

    opt_key = None

    argv = [arg for arg in sys.argv]  # take a copy
    # parse command line options and consume those we care about
    for arg in argv:
        if opt_key == 'compiler-flags':
            _cmake_extra_options.append(
                '-DCMAKE_CXX_FLAGS={arg}'.format(arg=arg.strip()))
        elif opt_key == 'exe-linker-flags':
            _cmake_extra_options.append(
                '-DCMAKE_EXE_LINKER_FLAGS={arg}'.format(arg=arg.strip()))
        elif opt_key == 'module-linker-flags':
            _cmake_extra_options.append(
                '-DCMAKE_MODULE_LINKER_FLAGS={arg}'.format(arg=arg.strip()))
        elif opt_key == 'G':
            _cmake_extra_options += ['-G', arg.strip()]
        elif opt_key == 'yes':
            _cmake_extra_options.append('-D{arg}=yes'.format(arg=arg.strip()))
        elif opt_key == 'no':
            _cmake_extra_options.append('-D{arg}=no'.format(arg=arg.strip()))
        elif opt_key == 'set':
            _cmake_extra_options.append('-D{arg}'.format(arg=arg.strip()))

        if opt_key:
            sys.argv.remove(arg)
            opt_key = None
            continue

        if arg == '--clean':
            _clean_build_folder = True
            sys.argv.remove(arg)
            continue

        if arg in ['--yes', '--no', '--set', '--compiler-flags',
                   '--exe-linker-flags', '--module-linker-flags']:
            opt_key = arg[2:].lower()
            sys.argv.remove(arg)
            continue
        if arg in ['-G']:
            opt_key = arg[1:]
            sys.argv.remove(arg)
            continue

    return _cmake_extra_options, _clean_build_folder


cmake_extra_options, clean_build_folder = get_extra_cmake_options()


class CMakeExtension(Extension):
    def __init__(self, name, sourcedir=''):
        Extension.__init__(self, name, sources=[])
        self.sourcedir = os.path.abspath(sourcedir)


def rmtree(name):
    """remove a directory and its subdirectories.
    """
    def remove_read_only(func, path, exc):
        excvalue = exc[1]
        if func in (os.rmdir, os.remove) and excvalue.errno == errno.EACCES:
            os.chmod(path, stat.S_IRWXU | stat.S_IRWXG | stat.S_IRWXO)
            func(path)
        else:
            raise Exception('Error while removing {}...'.format(path))

    if os.path.exists(name):
        log.info('Removing old directory {}'.format(name))
        shutil.rmtree(name, ignore_errors=False, onerror=remove_read_only)


class CMakeBuild(build_ext):

    def get_cmake_version(self):
        try:
            out = subprocess.check_output(['cmake', '--version'])
        except OSError:
            raise RuntimeError("\n*******************************************************************\n" +
                               " CMake must be installed to build the following extensions: " +
                               ", ".join(e.name for e in self.extensions) +
                               "\n*******************************************************************\n")
        return re.search(r'version\s*([\d.]+)', out.decode()).group(1)

    def run(self):
        cmake_version = self.get_cmake_version()
        if LooseVersion(cmake_version) < '3.10.0':
            raise RuntimeError("CMake >= 3.10.0 is required")

        for ext in self.extensions:
            self.build_extension(ext)

    def build_extension(self, ext):
        extdir = os.path.abspath(os.path.dirname(
            self.get_ext_fullpath(ext.name)))

        cmake_args = ['-DCMAKE_LIBRARY_OUTPUT_DIRECTORY=' + extdir,
                      '-DPYTHON_EXECUTABLE=' + sys.executable]

        cmake_args += cmake_extra_options

        cfg = 'Debug' if self.debug else 'Release'
        build_args = ['--config', cfg]

        cmake_args += ['-DCMAKE_BUILD_TYPE=' + cfg]
        # Do a parallel build
        build_args += ['--', '-j'+str(num_available_cpu_cores(2))]

        build_folder = os.path.abspath(self.build_temp)

        if clean_build_folder:
            rmtree(build_folder)
        if not os.path.exists(build_folder):
            os.makedirs(build_folder)

        cmake_setup = ['cmake', ext.sourcedir] + cmake_args
        cmake_build = ['cmake', '--build', '.'] + build_args

        print("Building extension for Python {}".format(
            sys.version.split('\n', 1)[0]))
        print("Invoking CMake setup: '{}'".format(' '.join(cmake_setup)))
        sys.stdout.flush()
        subprocess.check_call(cmake_setup, cwd=build_folder)
        print("Invoking CMake build: '{}'".format(' '.join(cmake_build)))
        sys.stdout.flush()
        subprocess.check_call(cmake_build, cwd=build_folder)


def num_available_cpu_cores(ram_per_build_process_in_gb):
    if 'TRAVIS' in os.environ and os.environ['TRAVIS'] == 'true':
        # When building on travis-ci, just use 2 cores since travis-ci limits
        # you to that regardless of what the hardware might suggest.
        return 2
    else:
        return 2


def read_version_from_cmakelists(cmake_file):
    """Read version information
    """
    with open(cmake_file) as f:
        text = f.read()
        match = re.findall(
            "set\\(arlib_VERSION (\\d+)\\.(\\d+)\\.(\\d+)\\)", text)[0]
        assert len(match) == 3
        major, minor, patch = match
    return major + '.' + minor + '.' + patch


setup(
    name='arlib',
    version=read_version_from_cmakelists('CMakeLists.txt'),
    description='A configurable, high-performance, alternative routing library.',
    long_description='See https://github.com/leonardoarcari/arlib for documentation.',
    author='Leonardo Arcari',
    author_email='leonardo1.arcari@gmail.com',
    url='https://github.com/leonardoarcari/arlib',
    license='MIT Licence',
    ext_modules=[CMakeExtension('arlib', 'tools/python')],
    cmdclass=dict(build_ext=CMakeBuild),
    zip_safe=False,
    keywords=['arlib', 'alternative route planning',
              'k shortest paths', 'alternative routing'],
)
