:: This script is meant to be run from within the Demos folder
if exist build\ (
rmdir /s /q build
)
mkdir build
cd build
cmake -G "MinGW Makefiles" ..
mingw32-make.exe -j6
cd ..
"%cd%"\build\Demo.exe