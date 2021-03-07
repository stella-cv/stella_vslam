@echo off

set openvslam_root="%~dp0\..\.."


echo =======================
echo Setup VCPKG
echo =======================

cd %openvslam_root%

cd 3rd

if exist vcpkg goto openvslam

git clone -b 2020.11-1 https://github.com/microsoft/vcpkg.git
cd vcpkg
call bootstrap-vcpkg.bat
call vcpkg.exe install g2o suitesparse yaml-cpp eigen3 glog opencv --triplet x64-windows-static



:openvslam

echo =======================
echo Setup OpenVSLAM
echo =======================

cd %openvslam_root%

mkdir Build
cd Build


cmake -S .. --preset=msvc-x64-static-debug
cmake -S .. --preset=msvc-x64-static-release