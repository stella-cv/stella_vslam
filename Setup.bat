@echo off


cd 3rd

if exist vcpkg goto dbow2

echo =======================
echo Setup VCPKG
echo =======================


git clone -b 2020.11-1 https://github.com/microsoft/vcpkg.git
cd vcpkg
call bootstrap-vcpkg.bat
call vcpkg.exe install g2o suitesparse yaml-cpp eigen3 glog opencv --triplet x64-windows-static
cd ..

:dbow2

if exist DBoW2 goto openvslam

echo =======================
echo Setup DBOW2
echo =======================

git clone https://github.com/OpenVSLAM-Community/DBoW2.git
cd DBoW2

mkdir Build
cd Build

cmake .. -DCMAKE_TOOLCHAIN_FILE="../../openvslam/3rd/vcpkg/scripts/buildsystems/vcpkg.cmake" -DVCPKG_TARGET_TRIPLET=x64-windows-static -DBUILD_SHARED_LIBS=OFF -DCMAKE_INSTALL_PREFIX="../Install/Debug"
cmake --build . --target install --config Debug

cmake .. -DCMAKE_TOOLCHAIN_FILE="../../openvslam/3rd/vcpkg/scripts/buildsystems/vcpkg.cmake" -DVCPKG_TARGET_TRIPLET=x64-windows-static -DBUILD_SHARED_LIBS=OFF -DCMAKE_INSTALL_PREFIX="../Install/Release"
cmake --build . --target install --config Release

cd ../..




:openvslam

pause

echo =======================
echo Build OpenVSLAM
echo =======================

git clone https://github.com/OpenVSLAM-Community/openvslam.git
cd openvslam


