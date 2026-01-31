@echo off
REM Build script that uses the WPILib 2026 JDK instead of system Java
REM This resolves compatibility issues between Java 25 and Gradle 8.11

set "JAVA_HOME=C:\Users\Public\wpilib\2026\jdk"
set "PATH=%JAVA_HOME%\bin;%PATH%"

echo Using WPILib JDK:
"%JAVA_HOME%\bin\java.exe" -version
echo.
echo Building project...
call gradlew.bat build

if %ERRORLEVEL% EQU 0 (
    echo.
    echo BUILD SUCCESSFUL
) else (
    echo.
    echo BUILD FAILED - Check errors above
)

pause
