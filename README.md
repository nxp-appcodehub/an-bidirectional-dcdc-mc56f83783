# NXP Application Code Hub
[<img src="https://mcuxpresso.nxp.com/static/icon/nxp-logo-color.svg" width="100"/>](https://www.nxp.com)

## AN14333: Bidirectional Resonant DC-DC Converter Design using MC56F83783

This is a Bidirectional Resonant DC-DC Converter Design using MC56F83783 DSC.

The MC56F83783 microcontroller is a member of the 32-bit 56800EX core-based Digital Signal Controllers (DSCs). Each device in the family combines the processing power of a 32-bit DSP and the functionality of a microcontroller with a flexible set of peripherals on a single chip. Due to its cost-effectiveness, configuration flexibility, and compact program code, 56F8xxxx is well-suited for many consumers and industrial applications.

Please refer to [AN14333](https://www.nxp.com/webapp/Download?colCode=AN14333&location=null&isHTMLorPDF=HTML) for complete instructions on how to use this software. 

#### Boards: Custom Board
#### Categories: Power Conversion
#### Peripherals: ADC, GPIO, PWM, TIMER
#### Toolchains: CodeWarrior

## Table of Contents
1. [Software](#step1)
2. [Hardware](#step2)
3. [Setup](#step3)
4. [Results](#step4)
5. [FAQs](#step5) 
6. [Support](#step6)
7. [Release Notes](#step7)

## 1. Software<a name="step1"></a>
- [SDK for MC56F83783: 2.13.1](https://mcuxpresso.nxp.com/en/builder?hw=MC56F83783&rel=677)
- [MCUXpresso Config Tools: v15](https://www.nxp.com/design/design-center/software/development-software/mcuxpresso-software-and-tools-/mcuxpresso-config-tools-pins-clocks-and-peripherals:MCUXpresso-Config-Tools)
- [CodeWarrior: v11.2](https://www.nxp.com/design/design-center/software/development-software/codewarrior-development-tools/downloads:CW_DOWNLOADS)
- [FreeMASTER: latest](https://www.nxp.com/design/design-center/software/development-software/freemaster-run-time-debugging-tool:FREEMASTER)

## 2. Hardware<a name="step2"></a>
- Power board: Bidirectional DCDC Converter Main Board Rev B 
- Control card: [HVP-56F83783](https://www.nxp.com/part/HVP-56F83783) Rev A or Rev A1
- Personal computer

## 3. Setup<a name="step3"></a>
1. Download the firmware to the control chip by connecting a debugger (P&E-Multilink) to SWD port of the control card using a 14-pin cable.
2. Plug in the control card, connect the DC source and load to setup the development enviroment, and using FreeMASTER by connecting the micro interface on the control card to the PC through a micro USB cable.
3. After DC source power on, select the desired working mode and run the converter in FreeMASTER, then the system starts working.

Please refer to [AN14333](https://www.nxp.com/webapp/Download?colCode=AN14333&location=null&isHTMLorPDF=HTML) chapter 5 for complete instructions on how to setup in different working modes.

![hardware](images/hardware.png)


## 4. Results<a name="step4"></a>
Please refer to [AN14333](https://www.nxp.com/webapp/Download?colCode=AN14333&location=null&isHTMLorPDF=HTML) chapter 5 for test results of this design.

## 5. FAQs<a name="step5"></a>
**5.1 Which operating modes does this design implement ?**

- BCM: Battery Charge Mode
- BDM: Battery Discharge Mode
- Working mode transition between BCM and BDM controlled by the FreeMASTER

**5.2 How does the DC-DC converter connect to the front-end AC-DC to work?**

- Connect the high voltage port to the DC bus port of the AC-DC.
- Connect the communication port. J6 on the DC-DC board is a reserved UART communication port for communication with the AC-DC.

**5.3 How to use FreeMASTER to control the system?**

Get a basic idea of FreeMASTER with [FreeMASTER User Guide](https://www.nxp.com/docs/en/user-guide/FMSTERUG.pdf). Go to [NXP community FreeMASTER Support Portal](https://community.nxp.com/t5/FreeMASTER/bd-p/freemaster) or [FreeMASTER landing page](https://www.nxp.com/freemaster) for more information if desired.

## 6. Support<a name="step6"></a>


#### Project Metadata
<!----- Boards ----->


<!----- Categories ----->
[![Category badge](https://img.shields.io/badge/Category-POWER%20CONVERSION-yellowgreen)](https://github.com/search?q=org%3Anxp-appcodehub+power_conversion+in%3Areadme&type=Repositories)

<!----- Peripherals ----->
[![Peripheral badge](https://img.shields.io/badge/Peripheral-ADC-yellow)](https://github.com/search?q=org%3Anxp-appcodehub+adc+in%3Areadme&type=Repositories) [![Peripheral badge](https://img.shields.io/badge/Peripheral-GPIO-yellow)](https://github.com/search?q=org%3Anxp-appcodehub+gpio+in%3Areadme&type=Repositories) [![Peripheral badge](https://img.shields.io/badge/Peripheral-PWM-yellow)](https://github.com/search?q=org%3Anxp-appcodehub+pwm+in%3Areadme&type=Repositories) [![Peripheral badge](https://img.shields.io/badge/Peripheral-TIMER-yellow)](https://github.com/search?q=org%3Anxp-appcodehub+timer+in%3Areadme&type=Repositories)

<!----- Toolchains ----->
[![Toolchain badge](https://img.shields.io/badge/Toolchain-CODEWARRIOR-orange)](https://github.com/search?q=org%3Anxp-appcodehub+codewarrior+in%3Areadme&type=Repositories)

Questions regarding the content/correctness of this example can be entered as Issues within this GitHub repository.

>**Warning**: For more general technical questions regarding NXP Microcontrollers and the difference in expected funcionality, enter your questions on the [NXP Community Forum](https://community.nxp.com/)

[![Follow us on Youtube](https://img.shields.io/badge/Youtube-Follow%20us%20on%20Youtube-red.svg)](https://www.youtube.com/@NXP_Semiconductors)
[![Follow us on LinkedIn](https://img.shields.io/badge/LinkedIn-Follow%20us%20on%20LinkedIn-blue.svg)](https://www.linkedin.com/company/nxp-semiconductors)
[![Follow us on Facebook](https://img.shields.io/badge/Facebook-Follow%20us%20on%20Facebook-blue.svg)](https://www.facebook.com/nxpsemi/)
[![Follow us on Twitter](https://img.shields.io/badge/Twitter-Follow%20us%20on%20Twitter-white.svg)](https://twitter.com/NXP)

## 7. Release Notes<a name="step7"></a>
| Version | Description / Update                           | Date                        |
|:-------:|------------------------------------------------|----------------------------:|
| 1.0     | Initial release on Application Code Hub        | June 25<sup>th</sup> 2024 |

