@echo off
chcp 65001 >nul
title Harta gropilor
cd /d "%~dp0server"

rem --- caut Python ---
set PY=
where py >nul 2>nul && set PY=py -3
if not defined PY ( where python >nul 2>nul && set PY=python )
if not defined PY (
  echo.
  echo  Python nu este instalat.
  echo  Descarca-l de la https://www.python.org/downloads/
  echo  La instalare bifeaza "Add python.exe to PATH", apoi porneste din nou acest fisier.
  echo.
  start https://www.python.org/downloads/
  pause
  exit /b 1
)

rem --- prima pornire: mediu virtual + Flask ---
if not exist ".venv\Scripts\python.exe" (
  echo Prima pornire: pregatesc mediul, dureaza un minut...
  %PY% -m venv .venv || goto :err
  ".venv\Scripts\python.exe" -m pip install --quiet --disable-pip-version-check -r requirements.txt || goto :err
)

".venv\Scripts\python.exe" run.py %*
pause
exit /b 0

:err
echo.
echo  A aparut o eroare la instalare. Verifica conexiunea la internet si incearca din nou.
rmdir /s /q .venv 2>nul
pause
exit /b 1
