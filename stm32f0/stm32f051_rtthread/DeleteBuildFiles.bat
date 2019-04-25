@echo off

rd .\MDK-ARM\12 /s /q
rd .\MDK-ARM\settings /s /q
rd .\MDK-ARM\stm32f103 /s /q
rd .\MDK-ARM\Objects /s /q
rd .\MDK-ARM\DebugConfig /s /q
rd .\MDK-ARM\Listings /s /q
del /Q .\MDK-ARM\*.Second


rd .\EWARM\12 /s /q
rd .\EWARM\settings /s /q
rd .\EWARM\12 /s /q


//del /Q *.dep
exit