@echo off
set argC=0
for %%x in (%*) do Set /A argC+=1

IF "%argC%"=="0" (
    call "%ProgramFiles%\Microsoft Visual Studio\2022\Community\Common7\Tools\VsDevCmd.bat"
)

set "current_path=%CD%"
for %%I in ("%current_path%") do set "folder_name=%%~nI"
echo The folder name is: %folder_name%
devenv %current_path%\%folder_name%\%folder_name%.vcxproj /build Debug
set RETVAL=%ERRORLEVEL%

if %RETVAL% equ 0 (
    echo "."
    echo " OOOOOOO Debug OK."
    echo "."
) else (
    echo " XXXXXXX Error. error code: %RETVAL%"
)

devenv %current_path%\%folder_name%\%folder_name%.vcxproj /build Release
set RETVAL=%ERRORLEVEL%

if %RETVAL% equ 0 (
    echo "."
    echo " OOOOOOO Release OK."
    echo "."
) else (
    echo " XXXXXXX Error. error code: %RETVAL%"
)