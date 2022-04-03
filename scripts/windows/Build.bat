@echo off

set param_1=%1

set stella_vslam_root="%~dp0\..\.."

cd %stella_vslam_root%

if "%param_1%" == "" (
    echo No option specified
    echo options are: 
    echo.
    echo setup      Build dependencies for stella_vslam
    echo debug      Build stella_vslam in debug mode
    echo release    Build stella_vslam in release mode
    echo develop    Setup Visual Studio solution file
    echo clean      Clean build and install directory
    echo.
    echo.

    set /p param_1=Option:
)

if "%param_1%" == "setup" (

    echo =======================
    echo Setup VCPKG
    echo =======================

    if not exist 3rd/FBoW (
        git submodule init
        git submodule update
    )

    cd 3rd

    git clone -b 2020.11-1 https://github.com/microsoft/vcpkg.git
    cd vcpkg
    call bootstrap-vcpkg.bat
    call vcpkg.exe install g2o suitesparse yaml-cpp eigen3 glog opencv --triplet x64-windows-static
)


if "%param_1%" == "debug" (

    echo =======================
    echo Debug build
    echo =======================


    mkdir build
    cd build
    
    cmake -S .. --preset=msvc-x64-static-debug

    cd msvc-x64-static-debug
    cmake --build . --target install --config Debug

)

if "%param_1%" == "release" (

    echo =======================
    echo Release build
    echo =======================

    mkdir build
    cd build

    cmake -S .. --preset=msvc-x64-static-release

    cd msvc-x64-static-release
    cmake --build . --target install --config Release
)

if "%param_1%" == "develop" (

    echo =======================
    echo Setup Visual Studio solution
    echo =======================

    mkdir build
    cd build

    cmake -S .. --preset=msvc-x64-static-debug
)

if "%param_1%" == "clean" (
    if exist build (
        rmdir /s/q build
    )
    if exist install (
        rmdir /s/q install
    )
)
