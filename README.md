# Taurus
Taurus is a 6dof tracking system capable of accurately measuring the position and orientation of PlayStation Move controllers in 3D space, allowing to do many things, such as VR, which this project also includes an example of in the form of a custom SteamVR driver communicating with Taurus.

# Calibration
Taurus has a built-in calibration tool, which is built as a separate build configuration. It compiles to the same directory as the main Taurus executable.
The program has written instructions for everything in it's CLI interface.

# Building and contributing
This project was designed to use the MSVC build pipeline and VCPKG for package management (eg. OpenCV, Protobuf). Written in VS 2022.
