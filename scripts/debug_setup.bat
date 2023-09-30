cd ..\Demos
if exist build\ (
rmdir /s /q build
)
mkdir build
cd build
cmake -G "MinGW Makefiles" .. -DCMAKE_BUILD_TYPE=Debug
mingw32-make.exe -j6