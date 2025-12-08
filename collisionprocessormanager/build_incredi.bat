@echo off
set argC=0
for %%x in (%*) do Set /A argC+=1
echo %argC%
IF "%argC%"=="0" (
    call "%ProgramFiles%\Microsoft Visual Studio\2022\Community\Common7\Tools\VsDevCmd.bat"
)
set ERRCNT=0
set "current_path=%CD%"
for %%I in ("%current_path%") do set "folder_name=%%~nI"
echo The folder name is: %folder_name%
echo "%current_path%\%folder_name%.sln"

rem devenv %current_path%\%folder_name%\%folder_name%.vcxproj /build Debug
BuildConsole.exe "%current_path%\%folder_name%.sln" /build /cfg="Debug|x64"
set RETVAL=%ERRORLEVEL%

if %RETVAL% equ 0 (
    echo "."
    echo " OOOOOOO Debug OK."
    echo "."
) else (
    echo " XXXXXXX Error. error code: %RETVAL%"
	Set /A ERRCNT+=1
)

rem devenv %current_path%\%folder_name%\%folder_name%.vcxproj /build Release
BuildConsole.exe "%current_path%\%folder_name%.sln" /build /cfg="Release|x64"

set RETVAL=%ERRORLEVEL%

if %RETVAL% equ 0 (
    echo "."
    echo " OOOOOOO Release OK."
    echo "."
) else (
    echo " XXXXXXX Error. error code: %RETVAL%"
	Set /A ERRCNT+=1
)

IF "%argC%"=="0" (
    pause
) else (
	if %ERRCNT% equ 0 (
		rem echo "RETURN 0"
		exit /b 0
	) else (
		rem echo "RETURN %ERRCNT%"
		exit /b %ERRCNT%
	)
)
