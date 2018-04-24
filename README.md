# TSOAX

## Installation

### Binaries

### From source

#### MacOS (>=10.8)

1. Install [Eigen 3](http://eigen.tuxfamily.org) and [Qt 5](https://www.qt.io) via [Homebrew](https://brew.sh):
   ``` bash
   $ brew install eigen qt
   ```
   Then add environment variables in your `.bash_profile`:
   ``` bash
   export Qt5_DIR=/usr/local/opt/qt
   export PATH=/usr/local/opt/qt/bin:$PATH
   ```
   and apply them: 
   ``` bash
   $ . ~/.bash_profile
   ```
2. Install VTK. Download [VTK 8.1.0](https://www.vtk.org/download/#latest) and do an out-of-source build:
   ``` bash
   $ mkdir your-vtk-build-dir
   $ cd your-vtk-build-dir
   $ cmake -DCMAKE_BUILD_TYPE=Release -DVTK_Group_Qt=ON /path/to/VTK-8.1.0/
   $ make -j 4
   ```
3. Build TSOAX.
   ``` bash
   $ git clone https://github.com/tix209/TSOAX.git
   $ cmake -DCMAKE_BUILD_TYPE=Release /path/to/tsoax/src/
   $ make -j 4
   ```

#### Linux

#### Windows
