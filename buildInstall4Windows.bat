rem
rem if QT version changes then the
rem Desktop_Qt_6_mm_n_MSVC_64bit will need
rem to chang to current version
rem
echo on
rem Save current directory
set ProjectDirTemp=%cd%

rem make sure the Qt environment is set
call C:\Qt\6.10.2\msvc2022_64\bin\qtenv2.bat

echo on
rem change back to original dir
cd /d %ProjectDirTemp%

mkdir build\Desktop_Qt_6_10_2_MSVC2022_64bit\Release\bundle
rem copy docs and license files to release directory
mkdir build\Desktop_Qt_6_10_2_MSVC2022_64bit\Release\bundle\docs

copy docs build\Desktop_Qt_6_10_2_MSVC2022_64bit\Release\bundle\docs
copy read*.md build\Desktop_Qt_6_10_2_MSVC2022_64bit\Release\bundle
copy "L2diagnostic LICENSE.txt" build\Desktop_Qt_6_10_2_MSVC2022_64bit\Release\bundle
copy "GNU V3 LICENSE.txt" build\Desktop_Qt_6_10_2_MSVC2022_64bit\Release\bundle
copy "Qt LICENSE LGPL.txt" build\Desktop_Qt_6_10_2_MSVC2022_64bit\Release\bundle
copy "Unitree BSD-3 LICENSE.txt" build\Desktop_Qt_6_10_2_MSVC2022_64bit\Release\bundle

rem copy all required dependencies to release directory
cd build\Desktop_Qt_6_10_2_MSVC2022_64bit\Release
copy L2diagnostic.exe bundle

cd bundle

windeployqt L2diagnostic.exe

rem  The Release/bundle folder contains the files for the complete release
rem  It should be copied into to a folder such as L2diagnosticVa-b-c
rem that folder zipped as the final distribution file
rem

rem change back to original dir
cd /d %ProjectDirTemp%
