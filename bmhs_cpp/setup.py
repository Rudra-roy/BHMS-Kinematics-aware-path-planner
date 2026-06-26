from pybind11.setup_helpers import Pybind11Extension, build_ext
from setuptools import setup

ext_modules = [
    Pybind11Extension(
        "bmhs_core",
        ["src/bmhs_core.cpp"],
        extra_compile_args=["-O3", "-std=c++17", "-pthread", "-march=native"],
        extra_link_args=["-pthread"],
    ),
]

setup(
    name="bmhs_core",
    version="1.0.0",
    description="C++ accelerated BMHS path planning core",
    ext_modules=ext_modules,
    cmdclass={"build_ext": build_ext},
)
