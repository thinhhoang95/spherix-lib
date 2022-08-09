# spherix-lib
The spherical codec for trajectory data

This is a C++ library for the Reference Spherical Codec (spherix-lib). 

Author: Thinh Hoang, Vincent Martinez, Daniel Delahaye, Pierre Maréchal.

# Build instructions

Load the project in cmake-gui.

Click "Configure".

Set the EIGEN3_INCLUDE_DIR to Eigen3's headers (see https://eigen.tuxfamily.org/dox/GettingStarted.html).

Set the installation path to wherever you like. We recommend /usr/share on Linux systems.

Click "Configure" again, all the red markings will be cleared.

Click Generate. Now open the generated project and build the BUILD target, it will automatically be installed to the selected installation path.

See the TestSampleTrajectory.cc file in the test folder for an example about how to use the library. In fact, spherix-lib should be used with Artery/Veins! Just link to the library in the installation path and include the headers in the "include" folder.

# License

The algorithm is patent pending, but the library is licensed under GPLv3.
