# pyutils26

This package provides utilities for working with instances and solutions of the
CG:SHOP 2026 competition. It includes functionality for reading and writing
instances and solutions, verifying solutions, and visualizing instances and
solutions.

## Installation

You can install the package via pip:

```bash
pip install cgshop2026-pyutils
```

Alternatively, you can install this package via source:

```bash
pip install --verbose .
```

You can also install it without the need to clone the repository:

```bash
pip install --verbose git+https://github.com/CG-SHOP/pyutils26
```

During the installation, CGAL and other dependencies will be downloaded and
compiled. This can take a while but should happen mostly automatic. You need to
have a C++ compiler (and the Python development environment) installed. Most
systems will come with all dependencies preinstalled. Otherwise, you can for
example install them on Ubuntu with the following command

```bash
sudo apt install build-essential python3-dev
```

You can test the installation with

```bash
pytest -s tests
```

> Please check for updates of the utils frequently as we are still working on
> them.

## Development

You can build the C++ of this project in place by `python setup.py develop`.
This will automatically take care of the conan dependencies and compile the C++
code. Then you can install the python package in editable mode by
`pip install -e .`. After that you only need to recompile the C++ code by
`python setup.py develop` if you change something in the C++ code.


## Changelog

- **0.1.0** (2025-10-14): Initial release