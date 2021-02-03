@echo off


cd Build/msvc-x64-static-release
cmake --build . --target install --config Release

cd ..
cd msvc-x64-static-debug
cmake --build . --target install --config Debug

pause