@echo off
rem Builds driver\handemu\bin\win64\driver_handemu.dll with the Visual Studio C++ toolchain.
rem Optional argument: another output folder (SteamVR locks the installed DLL while it runs).
setlocal
set "ROOT=%~dp0"

set "VSWHERE=%ProgramFiles(x86)%\Microsoft Visual Studio\Installer\vswhere.exe"
if not exist "%VSWHERE%" (
	echo vswhere.exe not found - install Visual Studio with the "Desktop development with C++" workload.
	exit /b 1
)
for /f "usebackq delims=" %%i in (`"%VSWHERE%" -latest -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath`) do set "VSDIR=%%i"
if not defined VSDIR (
	echo Visual Studio C++ tools not found.
	exit /b 1
)
call "%VSDIR%\VC\Auxiliary\Build\vcvars64.bat" >nul 2>&1 || exit /b 1

set "OUT=%ROOT%driver\handemu\bin\win64"
if not "%~1"=="" set "OUT=%~1"
set "OBJ=%ROOT%build\obj"
if not exist "%OUT%" mkdir "%OUT%"
if not exist "%OBJ%" mkdir "%OBJ%"

set "MH=%ROOT%third_party\minhook\src"
cl /nologo /c /O2 /MT /W3 /Fo"%OBJ%\\" "%MH%\hook.c" "%MH%\buffer.c" "%MH%\trampoline.c" "%MH%\hde\hde64.c" || exit /b 1

cl /nologo /O2 /MT /EHsc /std:c++17 /W3 /LD /DWIN32_LEAN_AND_MEAN /DNOMINMAX ^
	/I "%ROOT%third_party\openvr\headers" /I "%ROOT%third_party\minhook\include" ^
	/Fo"%OBJ%\\" /Fe"%OUT%\driver_handemu.dll" ^
	"%ROOT%src\driver.cpp" "%ROOT%src\gestures.cpp" "%ROOT%src\skeleton.cpp" "%ROOT%src\capture.cpp" ^
	"%OBJ%\hook.obj" "%OBJ%\buffer.obj" "%OBJ%\trampoline.obj" "%OBJ%\hde64.obj" ^
	/link /DLL /INCREMENTAL:NO || exit /b 1

del "%OUT%\driver_handemu.exp" "%OUT%\driver_handemu.lib" 2>nul
echo Built %OUT%\driver_handemu.dll
