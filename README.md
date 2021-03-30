##  Real time system I - (RTOS)

Personal resolution of the different practices required in the Subject.

Curse program ([link](https://sites.google.com/site/arquitecturademicros/)) 
LSE ([link](http://laboratorios.fi.uba.ar/lse/))
University ([link](http://www.fi.uba.ar/)) Universidad de Buenos Aires

## Supported targets:
- LPC1769
- LPC4337 (M4 and M0 cores)

## Supported boards:
- CIAA-NXP and EDU-CIAA-NXP (www.proyecto-ciaa.com.ar)
- LPCXpresso with LPC1769

## Supported toolchains:
- gcc-arm-none-eabi

## Usage
* Copy ```project.mk.template``` to ```project.mk```.
* Define ```PROJECT```, ```TARGET``` and ```BOARD``` (optional) variables in ```project.mk``` according to the project you want to compile.
* Compile with ```make```.
* Clean with ```make clean```.
* Download to target via OpenOCD with ```make download```.
