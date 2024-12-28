# setup.py

from distutils.core import setup, Extension
from Cython.Build import cythonize
import os

# option for generateing cython file
cythonize_options = {
    "language_level": 3,
    "annotate": False,
}

# change cython module file name
if os.path.exists("udp_module.cpp"):
    os.rename("udp_module.cpp", "udp_module_orig.cpp")

extensions = [
    Extension(
        "udp_module",
        sources=["udp_module.pyx", "udp_module_orig.cpp"],
        language="c++",
    )
]

setup(
    name="udp_module",
    ext_modules=cythonize(extensions, **cythonize_options),
)
