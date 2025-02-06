# TSOAX
TSOAX is an open source software to extract and track the growth and
deformation of biopolymer networks from 2D and 3D time-lapse sequences. It
tracks each filament or network branch from complex network dynamics and works
well even if filaments disappear or reappear.

TSOAX is an extension of [SOAX](https://www.lehigh.edu/~div206/soax/) (for
network extraction in static images) to network extraction and tracking in time
lapse movies.

If you find this repository helpful, please cite:
```
@article{XuTSOAX2019,
    author  = {T. Xu, C. Langouras, M. Adeli Koudehi, B. Vos, N. Wang,
        G. H. Koenderink, D. Vavylonis, and X. Huang},
    title   = {Automated Tracking of Biopolymer Growth and Network Deformation with TSOAX},
    journal = {Scientific Reports},
    year    = {2019},
}
```
You can access the full paper [here](https://rdcu.be/bl3ch).

## Installation
### Binaries
You can download and install binaries from [TSOAX
website](https://www.lehigh.edu/~div206/tsoax/).

This website also contains instructions on how to download TSOAX in a docker container

### From source

#### Ubuntu 22.04 (January 2025, for docker image)
1. Install the following libraries with apt-get: 

    build-essential  libeigen3-dev  qtbase5-dev  qtcreator  qt5-qmake
libqt5x11extras5-dev  cmake   x11-apps  cmake-gui
libxt-dev  libboost-all-dev

2. Compile VTK: 
    Start cmake-gui to compile VTK from source (8.2.0, https://github.com/Kitware/VTK/tree/v8.2.0) to a new VTK build directory 

    Chose Unix Makefile; VTK_Group_Qt: yes; BUILD_SHARED_LIBS: no; CMAKE_BUILD_TYPE: Release

    In VTK build directory: make -j 4

    To fix compiling errors: Add #include \<QPainterPath\> 
    in 
    "VTK-8.2.0\Rendering\Qt\vtkQtLabelRenderStrategy.h" and
    "VTK-8.2.0\Rendering\Qt\vtkQtStringToImage.h"

3. Compile TSOAX: 

    Get TSOAX: $git clone --recursive https://github.com/tix209/TSOAX.git

    cmake-gui to configure from TSOAX from source to a new TSOAX build directory

    Set VTK_DIR to VTK build path

    CMAKE_BUILD_TYPE Release

    Run make in TSOAX build directory. 


#### macOS (February 2022)

1.	Install XCode and eigen 3 with brew install eigen
2.	Install Qt5. This installation was checked with Qt 5.12.12 from download.qt.io/archive/qt/5.12/5.12.12
3.	Download CMake (GUI version) from cmake.org. This installation was checked with CMake 3.23.0
4.	Download VTK source code. This installation was checked with VTK 8.1.1 from https://github.com/Kitware/VTK/tree/v8.1.1
5.	Configure an Xcode VTK project with CMake. In CMake: 
select Release in CMAKE_CONFIGURATION_TYPES
activate Module_vtkGUISupportQt, Module_vtk_GUISupportQtOpenGL, VTK_Group_Qt
uncheck BUILD_SHARED_LIBS
set Qt5_DIR to the subfolder clang_64/lib/cmake/Qt5 of the Qt directory
6.	Compile VTK in XCode, selecting Intel and/or Silicon architectures
7.	Download TSOAX git clone --recursive https://github.com/tix209/TSOAX.git
8.	Configure an Xcode TSOAX project with CMake. In CMake:
set Qt5_DIR to the subfolder clang_64/lib/bin of the Qt directory
set VTK_DIR to the build  folder of VTK
9.	Compile a Release version of TSOAX in XCode, selecting Intel and/or Silicon architectures
(You may comment out including omp.h and delete -fopenmp as well update the include directory of eigen3 in the source)


#### Windows
1. Download and install [Microsoft Visual
   Studio](https://www.visualstudio.com/downloads/) 2015 or newer,
   [CMake](https://cmake.org), and [Qt 5](https://www.qt.io). This installation was checked with Qt version up to 5.12. 
2. Configure [VTK](https://www.vtk.org/download/#latest) with CMake (`cmake-gui`). This installation was checked with VTK 8.1.1.
   - Enter the folder path of where you extracted VTK source code
   - Enter another folder path for `vtk_binary_dir`
   - Click `Configure`
   - Specify generator e.g., Visual Studio 14 2015 Win64
   - After the configuration is done, check `VTK_Group_ENABLE_Qt` and `VTK_USE_CXX11_FEATURES`
   - Click `Configure`, and change `VTK_QT_VERSION` to 5 and
     `QT_QMAKE_EXECUTABLE` to `C:/Qt/5.10/msvc2015_64/bin`
   - Click `Configure`. Set `Qt5_DIR` (e.g. to `C:/Qt/5.12.2/msvc2017_64/lib/cmake5`)
   - Click `Configure`
   - Click `Generate` and then `Open Project`
   - In Visual Studio, change `Debug` to `Release` mode and build solution.
3. Download [Eigen 3](http://eigen.tuxfamily.org). Extract the zip file and rename it to `eigen3`.
4. For TSOAX version newer to 0.2.0, install the boost library (used by batch_tsoax) by downloading the Windows binaries. This installation was checked with boost version 1.77.0 
6. Open `Git Bash` then `$ git clone --recursive
   https://github.com/tix209/TSOAX.git`.
5. Configure TSOAX in CMake
   - Enter folder path of TSOAX source code
   - Enter folder path for `tsoax_binary_dir`
   - Click `Configure`
   - Specify generator e.g., Visual Studio 14 2015 Win64
   - Set `Qt5_DIR` , e.g. to `C:/Qt/5.12.2/msvc2017_64/lib/cmake/`
   - Click `Configure`. Set `VTK_DIR` to `vtk_binary_dir`
   - Change all /MD flags to /MT
   - If using boost, in CMake, set Boost_INCLUDE_DIR (e.g to C:/boost_1_77_0) and Boost_LIBRARY_DIR_RELEASE (e.g. to C:/boost_1_77_0/lib64-msvc-14.1)
   - Set Boost_FILESYSTEM_LIBRARY_DEBUG to C:/boost_1_77_0/lib64-msvc-14.1/libboost_filesystem-vc141-mt-sgd-x64-1_77.lib and Boost_FILESYSTEM_LIBRARY_RELEASE to C:/boost_1_77_0/lib64-msvc-14.1/libboost_filesystem-vc141-mt-s-x64-1_77.lib, or equivalent
   - Do the same for LIBRARY_DIR, PROGRAM_OPTIONS_LIBRARY and SYSTEM_LIBRARY, all of which point to the static libraries
   - Click `Configure`
   - Click `Generate` and then `Open Project`
   - In Visual Studio, change Debug to Release mode. Click `TSOAX` project and
   press <kbd>ALT</kbd>+<kbd>ENTER</kbd>. In `VC++ Directories`, add the path
   containing `eigen3` in `Include Directories`
   - Build the release version of TSOAX
   - Copy `Qt5Core.dll`, `Qt5Gui.dll`, `Qt5Widgets.dll` from `C:/Qt/5.8/msvc2015_64/bin/`
   to `tsoax_binary_dir/src/Release`.

#### Older compiling instructions for Linux and macOS (>=10.8)
1. Install [Eigen 3](http://eigen.tuxfamily.org) and [Qt 5](https://www.qt.io)
   using your package manager ([Homebrew](https://brew.sh) or dnf, apt):
   ``` bash
   $ brew install eigen qt@5 cmake # for macOS
   $ sudo dnf install eigen3-devel qt5-devel libXt-devel cmake # for Fedora
   ```
   (**macOS only**) Add environment variables in your `.bash_profile`:
   ``` bash
   export Qt5_DIR=/usr/local/opt/qt@5
   export PATH=/usr/local/opt/qt@5/bin:$PATH
   ```
   and apply them: 
   ``` bash
   $ . ~/.bash_profile
   ```
2. Install VTK. Download [VTK](https://www.vtk.org/download/#latest). 
   Do an out-of-source build
   ``` bash
   $ mkdir your-vtk-build-dir
   $ cd your-vtk-build-dir
   $ cmake -DCMAKE_BUILD_TYPE=Release -DVTK_GROUP_ENABLE_Qt=YES -DModule_vtkGUISupportQtOpenGL=ON /path/to/your-VTK-src-dir/
   $ make -j 4
   ```
   Add environment variable `VTK_DIR` in your `.bash_profile`:

   ``` bash
   export VTK_DIR=/path/to/your-vtk-build-dir
   ```
3. Build TSOAX.
   ``` bash
   $ git clone --recursive https://github.com/tix209/TSOAX.git
   $ cmake -DCMAKE_BUILD_TYPE=Release /path/to/tsoax/src/
   $ make -j 4
   ``` 
   In macOS, you can launch the program by searching for "TSOAX" in the
   spotlight (<kbd>⌘</kbd>+<kbd>Space</kbd>).


