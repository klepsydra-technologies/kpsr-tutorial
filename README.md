# kpsr-tutorial

<p align="right">
  <img width="25%" height="25%"src="./images/klepsydra_logo.jpg">
</p>

Public tutorial for using Klepsydra API. It consists of four chapters:

* [Chapter 1](./tutorials/chapter1.md) covers the core API of Klepsydra.
    * [Annex A](./tutorials/chapter1_annexA.md) covers how to pass a function as a parameter, paying special attention to different lambda function approaches.
    * [Annex B](./tutorials/chapter1_annexB.md) covers publisher and subscriber arguments.
* [Chapter 2](./tutorials/chapter2.md) covers the ZMQ connection API.
* [Chapter 3](./tutorials/chapter3.md) cover the code generator tool. It shows how to create a small application and build up to integrate with
middlewares like ZMQ.
* [Chapter 4](./tutorials/chapter4.md) covers the state machine.

Each chapter has its corresponding code in the [examples'](./examples) folder. Note, chapter 3 examples for basic code generator are in the core [examples/core](./examples/core) folder while the location of middleware specific examples is noted in the respective tutorial sections.

# Installation Instructions

## System dependencies

* Ubuntu 18.04 or above
* CMake 3.5.1 or above
* gcc for C++11 5.4.0 or above
* Google Tests (<https://github.com/klepsydra-technologies/googletest>)

Note that Google Tests is pulled in automatically when installing this project and does not need separate installation.

### Dependencies installed by Klepsydra

No separate installation is needed for the following dependencies if the Klepsydra installation was done correctly.

* [ConcurrentQueue](https://github.com/klepsydra-technologies/concurrentqueue)
* [Cereal](<https://github.com/klepsydra-technologies/cereal)
* [spdlog](<https://github.com/klepsydra-technologies/spdlog)

## Klepsydra dependencies

The following dependencies need to be installed:

* [kpsr-core](https://github.com/klepsydra-technologies/kpsr-core) optionally with ZMQ support
* [kpsr-codegen](https://github.com/klepsydra-technologies/kpsr-codegen)

`kpsr-build` will be pulled in automatically by the install process of this project.

### Installation

```bash
sudo apt install build-essential git cmake
```

Given `$KLEPSYDRA_HOME`, for example `$HOME/klepsydra`:

```bash
cd $KLEPSYDRA_HOME
git clone https://github.com/klepsydra-technologies/kpsr-tutorial
cd kpsr-tutorial/
git submodule update --init
mkdir build && cd build/
cmake ..
make -j$(nproc)
make test
```

The cmake has the following options:

* `-DCMAKE_PREFIX_PATH=<path>` Klepsydra SDK installation location (`/usr/local` by default), same as `-DCMAKE_INSTALL_PREFIX` when building kpsr-core
* `-DKPSR_INSTALL_PATH=<path>` to specify where kpsr-tutorial binaries should be installed (`/opt/klepsydra` by default)
* `-DKPSR_WITH_ZMQ=true` for the ZMQ examples

The examples binaries, such as `kpsr_tutorial_chapter1_1`, are located at `build/bin/`

# License

&copy; 2023 Klepsydra Technologies AG, all rights reserved. Licensed under the terms in [LICENSE.md](./LICENSE.md)

This software and documentation are 2023 Klepsydra Technologies AG
Limited and its licensees. All rights reserved. See [license file](./LICENSE.md) for full copyright notice and license terms.

# Contact

<https://www.klepsydra.com>
support@klepsydra.com
