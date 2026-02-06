echo on
rem Save current directory
set ProjectDirTemp=%cd%

rem make sure the Qt environment is set
call C:\Qt\6.10.1\msvc2022_64\bin\qtenv2.bat

echo on
rem change back to original dir
cd /d %ProjectDirTemp%

rem copy docs and license files to release directory
mkdir build\Desktop_Qt_6_10_1_MSVC2022_64bit\Release\docs

copy docs build\Desktop_Qt_6_10_1_MSVC2022_64bit\Release\docs
copy read*.md build\Desktop_Qt_6_10_1_MSVC2022_64bit\Release

rem copy all required dependencies to release directory
cd build\Desktop_Qt_6_10_1_MSVC2022_64bit\Release

windeployqt L2diagnostic.exe

rem change back to original dir
cd /d %ProjectDirTemp%
