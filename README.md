<p align="right">
  <img width="25%" height="25%"src="./images/klepsydra_logo.jpg">
</p>

# kpsr-tutorial

Public tutorial for using Klepsydra API. It consists of three chapters:

* [Chapter 1](./tutorials/chapter1.md) covers the core API of Klepsydra.
* [Chapter 2](./tutorials/chapter2.md) covers the ROS and DDS connection API.
* [Chapter 3](./tutorials/chapter3.md) cover the code generator tool. It shows how to create a small application and build up to integrate with
middlewares like ROS, ZMQ or DDS.
* [Chapter 4](./tutorials/chapter4.md) covers the state machine.

Each chapter has its corresponding code in the [examples](./examples) folder. Note, chapter 3 examples for basic code generator are in the core [examples/core](./examples/core) folder while the location of middleware specific examples is noted in the respective tutorial sections.

# Installation Instructions

## System dependencies

* Ubuntu 18.04 or above
* ROS Indigo or above (optional)
* DDS (optional)
* CMake 3.5.1 or above
* gcc for C++11 5.4.0 or above.
* Google Tests (<https://github.com/klepsydra-technologies/googletest>)

Note that Google Tests is pulled in automatically when installing this project and does not need separate installation.

### Dependencies installed by Klepsydra

No separate installation is needed for the following dependencies if the Klepsydra installation was done correctly.

* ConcurrentQueue (<https://github.com/klepsydra-technologies/concurrentqueue>)
* Cereal (<https://github.com/klepsydra-technologies/cereal>)
* spdlog (<https://github.com/klepsydra-technologies/spdlog>)

## Klepsydra dependencies

* [kpsr-core](https://github.com/klepsydra-technologies/kpsr-core) (installed with yaml support)
* [kpsr-build](https://github.com/klepsydra-technologies/kpsr-build)

kpsr-build will be pulled in automatically by the install process of this project.

### Installation

If you wish to follow the DDS and ROS examples in this
project, then the Klepsydra software must have been installed using the
`-DKPSR_WITH_DDS=true` and `-DKPSR_WITH_ROS=true` arguments with cmake.

```bash
sudo apt install build-essentials git cmake
```

Given `$KLEPSYDRA_HOME`, for example `$HOME/klepsydra`:

```bash
cd $KLEPSYDRA_HOME
git clone https://github.com/klepsydra-technologies/kpsr-tutorial
cd kpsr-tutorial
git submodule update --init
mkdir build
cd build
cmake ..
make
make test
```

The cmake has the following options:

* -DCMAKE_PREFIX_PATH Klepsydra SDK installation location (`/usr/local` by default), same as -DCMAKE_INSTALL_PREFIX when building kpsr-core
* -DKPSR_INSTALL_PATH to specify where kpsr-tutorial binaries should be installed (`/opt/klepsydra` by default)
* -DKPSR_WITH_DDS for building the DDS version of the tutorial

The examples binaries, such as `kpsr_tutorial_chapter1_1`, are located at `build/bin/`
<!--- FIXME make install? -->

### Install process for ROS Examples

For compiling ROS examples, we further need to install the kpsr-tutorial module. This can be done by running (after the above build process):

```bash
sudo make install
```

Next, make a symlink in your ROS catkin workspace to point to  the ROS subfolder of this project.

```bash
cd $CATKIN_WORKSPACE/src
source /opt/ros/melodic/setup.bash (alt: source /opt/ros/kinetic/setup.bash)
ln -s $KLEPSYDRA_HOME/kpsr-tutorial/examples/chapter/kpsr_tutorial_chp2_ros
catkin_make
```

To execute the tests:

```bash
catkin_make tests
catkin_make run_tests_kpsr_ros_tutorial
```

Run the ROS example as you would run any ros node:

```bash
source devel/setup.bash
rosrun kpsr_tutorial_chp2_ros kpsr_tutorial_chp2_ros_ex2
```

# License

&copy; 2023 Klepsydra Technologies AG, all rights reserved. Licensed under the terms in [LICENSE.md](./LICENSE.md)

This software and documentation are 2023 Klepsydra Technologies AG
Limited and its licensees. All rights reserved. See [license file](./LICENSE.md) for full copyright notice and license terms.

# Contact

<https://www.klepsydra.com>
support@klepsydra.com
