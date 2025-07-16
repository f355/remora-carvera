# Remora for Carvera

This is a fork of fantastic [Remora](https://github.com/scottalford75/Remora) for use with the Carvera family of machines by [Makera Inc](https://www.makera.com/).
It is heavily work in progress (almost nothing works!) and would likely support only Carvera Air in the foreseeable future, as I (@f355) don't have the big-brother Carvera.

The Remora authors [say they "dont ...not support"](https://github.com/scottalford75/Remora/issues/78#issuecomment-2584956914) LPC1768-based boards,
so this is more of a hard-fork, the changes are not intended to be upstreamed.

# Building

Due to historical reasons, the project is using Mbed OS 5.x which is pretty tricky to get working with any sensible build tooling -
the Mbed CLI drops into a dependency hell on modern systems.

Don't bother, just download the Mbed Studio, open this directory, choose the `mbed LPC1768` target and the `Release` profile, then build.


# Remora - the original README

The full documentation is at <https://remora-docs.readthedocs.io/en/latest/>
Note: Docs have not been updated for 1.0.0_rc

Remora is a free, opensource LinuxCNC component and Programmable Realtime Unit (PRU) firmware to allow LPC176x and STM32F4 based controller boards to be used in conjuction with a Raspberry Pi to implement a LinuxCNC based CNC controller.

Having a low cost and accessable hardware platform for LinuxCNC is important if we want to use LinuxCNC for 3D printing for example. Having a controller box the size of the printer itself makes no sense in this applicatoin. A SoC based single board computer is ideal in this application. Although developed for 3D Printing, Remora (and LinuxCNC) is highly flexible and configurable for other CNC applications.

Remora has been in use amd development since 2017. Starting on Raspberry Pi 3B and 3B+ eventhough at the time it was percieved that the Raspberry Pi was not a viable hardware for LinuxCNC.

With the release of the RPi 4 the LinuxCNC community now supports the hardware, with LinuxCNC and Preempt-RT Kernel packages now available from the LinuxCNC repository. This now greatly simplifies the build of a Raspberry Pi based CNC controller.
