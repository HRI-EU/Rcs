# Rcs

Rcs is a set of C and C++ libraries for robot control and simulation. It is written for research purposes and for simulation and analysis in robotics. It contains algorithms for vector-matrix calculations, rigid body kinematics, dynamics, control, physical simulation and more. There are many utilities such as OpenSceneGraph nodes and Qt Guis to support research and testing. 

<p float="left">
<img src="doc/images/asimo-sim-1.5.png" width="19%" > 
<img src="doc/images/darwin.jpg" width="19%" > 
<img src="doc/images/valkyrie.jpg" width="19%" > 
<img src="doc/images/atlas.jpg" width="19%" > 
<img src="doc/images/humanoid.jpg" width="19%" >
</p>

<p float="left">
<img src="doc/images/wam.jpg" width="19%" > 
<img src="doc/images/jaco.jpg" width="19%" > 
<img src="doc/images/husky.jpg" width="19%" > 
<img src="doc/images/lwa.jpg" width="19%" > 
<img src="doc/images/pa10.jpg" width="19%" > 
</p>

<p float="left">
<img src="doc/images/face.jpg" width="19%" > 
<img src="doc/images/ergo.jpg" width="19%" > 
<img src="doc/images/mobile-manipulator.jpg" width="19%" >
<img src="doc/images/pa10-creature.jpg" width="19%" > 
<img src="doc/images/WAM-bulb.png" width="19%" >
</p>

## How to compile

### Linux

Rcs can be compiled with the cmake build system and has mainly been developed on Ubuntu 14.04 and GCC 4.8. To compile it, just type:

    cd <build-directory>
    cmake <source-directory>
    make 
    make rcsunittest

To build the doxygen documentation, just type:

    make RcsDoc

Compilation has successfully been tested on Ubuntu 14.04, Ubuntu 16.04 with GCC 5 and Ubuntu 18.04 with GCC 7 and clang, and Ubuntu 20.

Note that using Vortex Essentials on newer operating systems requires extra care. The official distribution is compiled with GCC 4.8, and will not work with newer compiler versions. To work around this limitation, Rcs compiles the Vortex integration module separately. If GCC 4.8 is available (by installing the `g++-4.8` package), the integration module is built automatically. If it isn't, you need to provide a pre-built version of libRcsVortex.so.

### Windows native with Visual Studio

We recommend to use the vcpkg package manager to obtain the dependencies. These packages need to be installed:

```
me@computer: > vcpkg.exe install pthreads libxml2 osg qwt 
```

These are optional packages that extend the functionality of Rcs:
```
me@computer: > vcpkg.exe install bullet3 eigen3 octomap
```


This is how to run the cmake command for an out-of-source build directory:

```
me@computer: > cd <build-directory> && cmake.exe -G "Visual Studio 15 2017 Win64" -DCMAKE_WINDOWS_EXPORT_ALL_SYMBOLS=TRUE -DCMAKE_BUILD_TYPE=Release -DVCPKG_TARGET_TRIPLET=x64-windows-release -DCMAKE_TOOLCHAIN_FILE=C:\dev\vcpkg\scripts\buildsystems\vcpkg.cmake <Rcs root directory> -Wno-dev
```

It is convenient to put these instructions into a script file:

```
cd <build-directory>
cmake.exe -G "Visual Studio 15 2017 Win64" -DCMAKE_WINDOWS_EXPORT_ALL_SYMBOLS=TRUE -DCMAKE_BUILD_TYPE=Release -DVCPKG_TARGET_TRIPLET=x64-windows-release -DCMAKE_TOOLCHAIN_FILE=C:\dev\vcpkg\scripts\buildsystems\vcpkg.cmake <Rcs root directory> -Wno-dev
cd ..
set /p=Hit ENTER to continue...
```

The Rcs CmakeLists.txt contain targets that copy the Windows dlls to the build directory. It is therefore not necessary that the paths to the dlls are added to the environment variables. The above examples might require adjusting to your particular settings (e.g. Visual Studio version or VCPKG_TARGET_TRIPLET ...).

## Build options

 - USE_BULLET: Enable bullet physics. With this option, a PhysicsSimulation class that uses Bullet Physics will be built. It can be instantiated through the PhysicsFactory class. Please refer to the documentation. Rcs requires a version equal or higher than 2.83, compiled with shared libraries. Please refer to the Bullet Physics instructions. In case cmake can find a bullet library, the option is ON, otherwise it is OFF.
 - USE_WM5: Use the GemoetricTools library (WildMagic5). This will enable a number of functions related to computational geometry, e.g. distance calculation of shape primitives, and ray casts. In case cmake can find the library, the option is ON, otherwise it is OFF.
 - USE_VORTEX: Enable Vortex physics. With this option, a PhysicsSimulation class that uses Vortex Essentials (CmLabs) or Vortex with version 6.8 will be built. It can be instantiated through the PhysicsFactory class. Please refer to the documentation. To obtain Vortex, please refer to the CmLabs web-site. The default is OFF.
 - VORTEX_ESSENTIALS_DIR: Installation directory of Vortex Essentials. Required since there is no standard location for this.
 - USE_EIGEN3: Use the Eigen3 library. This will compile in a number of additional linear algebra functions (See Rcs_eigen3.h for details)
 - USE_OCTOMAP: Use the OctoMap library. This will compile in a number of additional distance functions to copmpute distances of the geometric shapes against octrees.
 - ENABLE_DEBUG_MACROS: This option enables logging macros that allow to log information on different debug levels (such as RLOG). If disabled, all these conditional logs are not compiled into the binaries. This usually leads to faster programs, however with the drawback of loosing the information if something goes wrong. The default is ON.
 - ENABLE_C++11: This option sets the corresponding flag so that all code is compiled with the C++11 support. It is not needed from the code, but might be necessary for binary compatibility if other code is compiled for C++11. The default is OFF.
 - HEADLESS_BUILD: Build only RcsCore and RcsPhysics, and leave out all graphics and Gui related classes. If no graphics and Gui libraries are present on the system, you need to call cmake -DHEADLESS_BUILD=TRUE. The default is OFF.

## Project structure

  - bin :             Source files with a main() function
  - cmake :           CMake related configuration files
  - config :          Configuration files such as e.g xml, 
  - doc :             Documentation (doxygen configuration, latex)
  - examples :        Examples with a main() function
  - external :        Third party source code
  - src :             Source files that are compiled into libraries
    - RcsCore :       Algorithms, math, utilities
    - RcsGui :        Qt widgets
    - RcsGraphics :   OpenSceneGraph nodes
    - RcsPhysics :    Bullet and Vortex wrappers

## 3rd party libraries

Rcs has been designed carefully to have only little dependencies. The ones that have been selected are in our opinion very matured libraries:

 - Libxml2 (MIT License, Ubuntu-package libxml2-dev)
 - Qt5: LGPL (Also dual commercial license available, Ubuntu-package qt5-default)
 - qwt (LGPL with additions, Ubuntu-package libqwt-qt5-dev)
 - OpenSceneGraph (OSGPL, Ubuntu-package libopenscenegraph-dev)

Optionally (please use the ccmake tool to manage the compile options), additional functionality can be compiled in when activating

 - GeometricTools (Wild Magic Engine 5.17, Boost Software License 1.0). Web-site: [https://www.geometrictools.com](https://www.geometrictools.com)
 - Bullet Physics (Zlib, Ubuntu-package libbullet-dev)
 - Vortex Studio Essentials
 - Eigen3 (Mozilla Public License Version 2.0)
 - OctoMap (New BSD License). Web-site: [http://octomap.github.io](http://octomap.github.io)

In case you don't have a Vortex license, you can in certain cases apply for the Adademic Access Program that allows to get a free license. Please check: [https://www.cm-labs.com](https://www.cm-labs.com)

## Troubleshooting

  - Bullet library linking errors: In case you use a bullet version compiled with double precision, please make sure the define BT_USE_DOUBLE_PRECISION is used during compilation (see cmake/Externals.cmake for details). Likewise, in case you are using a bullet library compiled with float values, make sure that the above define is not set.

## License

This project is licensed under the BSD 3-clause license - see the [LICENSE.md](LICENSE.md) file for details

## Disclaimer

The copyright holders are not liable for any damage(s) incurred due to improper use of Rcs.

## TODO

- void Rcs_setIKState(RcsGraph* self, const double* x) obsolete?
- Strange in Rcs_gradientTests.c:
  RCSGRAPH_TRAVERSE_BODIES(self)
  {
    if (BODY->jntId != -1)
    {
      b = BODY;
    }
  }
