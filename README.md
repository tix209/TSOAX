# TSOAX
TSOAX is an open source software to extract and track the growth and
deformation of biopolymer networks from 2D and 3D time-lapse sequences. It
tracks each filament or network branch from complex network dynamics and works
well even if filaments disappear or reappear.

TSOAX is an extension of [SOAX](http://athena.physics.lehigh.edu/soax/) (for
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
## Installation
### Binaries
You can download and install binaries from [TSOAX
website](http://athena.physics.lehigh.edu/tsoax/).

### From source

#### Linux and macOS (>=10.8)
1. Install [Eigen 3](http://eigen.tuxfamily.org) and [Qt 5](https://www.qt.io)
   using your package manager ([Homebrew](https://brew.sh) or dnf, apt):
   ``` bash
   $ brew install eigen qt  # for macOS
   $ sudo dnf install eigen3-devel qt5-devel libXt-devel  # for Fedora
   ```
   (**macOS only**) Add environment variables in your `.bash_profile`:
   ``` bash
   export Qt5_DIR=/usr/local/opt/qt
   export PATH=/usr/local/opt/qt/bin:$PATH
   ```
   and apply them: 
   ``` bash
   $ . ~/.bash_profile
   ```
2. Install VTK. Download [VTK 8.1.0](https://www.vtk.org/download/#latest) and
   do an out-of-source build
   ``` bash
   $ mkdir your-vtk-build-dir
   $ cd your-vtk-build-dir
   $ cmake -DCMAKE_BUILD_TYPE=Release -DVTK_Group_Qt=ON /path/to/VTK-8.1.0/
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

#### Windows
1. Download and install [Microsoft Visual
   Studio](https://www.visualstudio.com/downloads/) 2015 or newer,
   [CMake](https://cmake.org), and [Qt 5](https://www.qt.io).
2. Configure [VTK 8.1.0](https://www.vtk.org/download/#latest) with CMake (`cmake-gui`).
   - Enter the folder path of where you extracted VTK source code
   - Enter another folder path for `vtk_binary_dir`
   - Click `Configure`
   - Specify generator e.g., Visual Studio 14 2015 Win64
   - After the configuration is done, check `VTK_Group_Qt` and `VTK_USE_CXX11_FEATURES`
   - Click `Configure`, and change `VTK_QT_VERSION` to 5 and
     `QT_QMAKE_EXECUTABLE` to `C:/Qt/5.10/msvc2015_64/bin`
   - Click `Configure`. Change `Qt5_DIR` to `C:/Qt/5.8/msvc2015_64/lib/cmake/Qt5`
   - Click `Configure`
   - Click `Generate` and then `Open Project`
   - In Visual Studio, change `Debug` to `Release` mode and build solution.
3. Download [Eigen 3](http://eigen.tuxfamily.org). Extract the zip file and rename it to `eigen3`.
4. Open `Git Bash` then `$ git clone --recursive
   https://github.com/tix209/TSOAX.git`.
5. Configure TSOAX in CMake
   - Enter folder path of TSOAX source code
   - Enter folder path for `tsoax_binary_dir`
   - Click `Configure`
   - Specify generator e.g., Visual Studio 14 2015 Win64
   - Set `Qt5_DIR` to `C:/Qt/5.8/msvc2015_64/lib/cmake/Qt5Widgets`
   - Click `Configure`. Set `VTK_DIR` to `vtk_binary_dir`
   - Click `Configure`
   - Click `Generate` and then `Open Project`
   - In Visual Studio, change Debug to Release mode. Click `TSOAX` project and
   press <kbd>ALT</kbd>+<kbd>ENTER</kbd>. In `VC++ Directories`, add the path
   containing `eigen3` in `Include Directories`
   - Build the release version of TSOAX
   - Copy `Qt5Core.dll`, `Qt5Gui.dll`, `Qt5Widgets.dll` from `C:/Qt/5.8/msvc2015_64/bin/`
   to `tsoax_binary_dir/src/Release`.
