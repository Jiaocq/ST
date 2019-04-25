@echo off

rd .\12 /s /q
rd .\settings /s /q
rd .\stm32f103 /s /q
rd .\Objects /s /q
rd .\DebugConfig /s /q
rd .\Listings /s /q
del /Q *.Second
//del /Q *.dep
exit