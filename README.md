# Rigid Body Dynamics for Robotic Applications
RBDYN is a C++ library which contains rigid body dynamics algorithms such as the Articulated Body Algorithm (ABA) for forward dynamics, Recursive Newton-Euler Algorithm (RNEA) for inverse dynamics and the Composite Rigid Body Algorithm (CRBA) for the computation of the joint space inertia matrix.

The library requires additional library called [TinyXML2](https://github.com/leethomason/tinyxml2) to compile.
Also, [RaiSim](www.raisim.com) is required to run examples. So, if you want to run examples first download RaiSim.

## Installing and Compiling TinyXML2 with `-fPIC`
TinyXML2 is a lightweight XML parser for C++. This guide explains how to **manually download, build, and install TinyXML2** with `-fPIC` to ensure compatibility with shared libraries (`.so`).

---

##  **1. Download TinyXML2 Source Code**
Clone the official [TinyXML2](https://github.com/leethomason/tinyxml2) repository from GitHub:
```sh
git clone https://github.com/leethomason/tinyxml2.git
```
Build with CMAKE:
```sh
cd tinyxml2
mkdir build && cd build
cmake .. -DCMAKE_POSITION_INDEPENDENT_CODE=ON
cmake --build .
```
Install the library:
```sh
sudo cmake --install .
```
##  **2. Download RBDYN Source Code**
Clone the [RBDYN](https://github.com/erimcanozcinar/rbdyn) repository from GitHub:
```sh
git clone https://github.com/erimcanozcinar/rbdyn.git
```
Build with CMAKE:
```sh
cd rbdyn
mkdir build && cd build
cmake ..
make
```
If you do not want to run examples. Just compile the library:
```sh
make rbdyn
```
Install library:
```sh
sudo make install
```
