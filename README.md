# libhal-mpl

[![✅ Checks](https://github.com/libhal/libhal-mpl/actions/workflows/ci.yml/badge.svg)](https://github.com/libhal/libhal-mpl/actions/workflows/ci.yml)
[![Coverage](https://libhal.github.io/libhal-mpl/coverage/coverage.svg)](https://libhal.github.io/libhal-mpl/coverage/)
[![Codacy Badge](https://app.codacy.com/project/badge/Grade/b084e6d5962d49a9afcb275d62cd6586)](https://www.codacy.com/gh/libhal/libhal-mpl/dashboard?utm_source=github.com&amp;utm_medium=referral&amp;utm_content=libhal/libhal-mpl&amp;utm_campaign=Badge_Grade)
[![GitHub stars](https://img.shields.io/github/stars/libhal/libhal-mpl.svg)](https://github.com/libhal/libhal-mpl/stargazers)
[![GitHub forks](https://img.shields.io/github/forks/libhal/libhal-mpl.svg)](https://github.com/libhal/libhal-mpl/network)
[![GitHub issues](https://img.shields.io/github/issues/libhal/libhal-mpl.svg)](https://github.com/libhal/libhal-mpl/issues)

libhal compatible device library for the MPL3115A2 device.

## Contributing

See [`CONTRIBUTING.md`](CONTRIBUTING.md) for details.

## License

Apache 2.0; see [`LICENSE`](LICENSE) for details.

## Disclaimer

This project is not an official Google project. It is not supported by
Google and Google specifically disclaims all warranties as to its quality,
merchantability, or fitness for a particular purpose.

## About the libhal-device template

The libhal-__device__ repository is a template for creating device libraries in
the libhal ecosystem. It provides a structured layout and a set of files to help
you get started with creating your own device library.

## .github/workflows

This directory contains GitHub Actions workflow files for continuous integration
(CI) and other automated tasks. The workflows currently included are:

- `ci.yml`: This workflow runs the CI pipeline, which includes
  building the project, running tests, and deploying the library to the
  `libhal-trunk` package repository.
- `take.yml`: This workflow is responsible for the "take" action, which assigns
  commits to
- `update_name.yml`: This workflow updates the name of the repository when it's
  used as a template for a new repository.

## conanfile.py

This is a [Conan](https://conan.io/) recipe file. Conan is a package manager for
C and C++ that helps manage dependencies in your project. This file defines how
Conan should build your project and its dependencies.

## datasheets

This directory is intended for storing datasheets related to the device that the
library is being built for. It currently contains a placeholder file,
`put_datasheets_here.md`.

Many datasheets are subject to copyright and that must be considered when adding
the datasheet to a libhal repo. If the datasheet cannot be redistributed on the
repo for copyright and/or license reasons, then a markdown file with a link to
the datasheet (and potentially mirrors of it) is an acceptable alternative.

## demos

This directory contains demonstration applications showing how to use the device
library. It includes:

- `applications/MPL3115A2.cpp`: A sample application demonstrating usage of the
  device library.
- `hardware_map.hpp`: A header file defining the hardware map for the demo
  applications.
- `main.cpp`: The main entry point for the demo applications.
- `platforms/lpc4074.cpp` and `platforms/lpc4078.cpp`: Platform-specific
  implementations for the demo applications.

## include/libhal-mpl

This directory contains the header files for the device library. It currently
includes `mpl.hpp`, which is a placeholder for the main header file of
your device library.

## src

This directory contains the source files for the device library. It currently
includes `mpl.cpp`, which is a placeholder for the main source file of
your device library.

## test_package

This directory contains a test package for the Conan recipe. It includes a
simple application that uses the device library, which helps verify that the
Conan recipe is working correctly.

## tests

This directory contains tests for the device library. It includes:

- `mpl.test.cpp`: A placeholder for tests for the device library.
- `main.test.cpp`: The main entry point for the tests.

Remember to replace all instances of `mpl` with the actual name of the
device that your library is being built for.
