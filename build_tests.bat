@echo off
rem Builds the driver into build\stage, then runs it inside the fake vrserver (tests\mock_vrserver.cpp).
rem The staged DLL does not touch the installed one, so this works while SteamVR is running.
setlocal
set "ROOT=%~dp0"
set "STAGE=%ROOT%build\stage"
call "%ROOT%build.bat" "%STAGE%" || exit /b 1

set "VSWHERE=%ProgramFiles(x86)%\Microsoft Visual Studio\Installer\vswhere.exe"
for /f "usebackq delims=" %%i in (`"%VSWHERE%" -latest -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath`) do set "VSDIR=%%i"
call "%VSDIR%\VC\Auxiliary\Build\vcvars64.bat" >nul 2>&1 || exit /b 1

set "OBJ=%ROOT%build\test_obj"
if not exist "%OBJ%" mkdir "%OBJ%"

cl /nologo /O2 /MT /EHsc /std:c++17 /W3 /I "%ROOT%third_party\openvr\headers" ^
	/Fo"%OBJ%\\" /Fe"%ROOT%build\skeleton_test.exe" "%ROOT%tests\skeleton_test.cpp" "%ROOT%src\skeleton.cpp" || exit /b 1
"%ROOT%build\skeleton_test.exe" || exit /b 1

rem /Od and /OPT:NOICF keep every fake interface method a separate, hookable function.
cl /nologo /Od /MT /EHsc /std:c++17 /W3 /I "%ROOT%third_party\openvr\headers" ^
	/Fo"%OBJ%\\" /Fe"%ROOT%build\mock_vrserver.exe" "%ROOT%tests\mock_vrserver.cpp" ^
	/link /OPT:NOICF /OPT:NOREF /INCREMENTAL:NO || exit /b 1

"%ROOT%build\mock_vrserver.exe" "%STAGE%\driver_handemu.dll"
