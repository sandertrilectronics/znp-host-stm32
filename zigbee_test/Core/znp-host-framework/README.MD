# ZNP Host Framework

## Introduction

This git repository contains the source code for the ZNP POSIX Host Framework and Examples. The project is intended to help developer to develop ZigBee application on a host MCU/MPU connected to the Texas Instruments ZigBee Network Processor (CC253x). The framework offers a C callable / callback API to access the ZigBee Network Processor functionality available as a Binary Remote Procedure Call format over the UART/SPI interface.

The Framework and Examples utilize a subset of POSIX functions to access OS functionality (Semaphores etc) making it easily portable between OS’s (High Level or Real Time) and HW platforms.

The Examples currently implemented are:

-	CmdLine: A command line example that exposes the ZNP API on the command line, auto complete and help on the available commands is at hand by pressing TAB. This example allows the user to intuitively learn the ZNP interface API as well as ZigBee commands. 
-	DataSendRecv: A Simple example to send / receive data between nodes in the network.
-	NwkTopology: A Simple example to discover the topology of a network that the node is a part of.
-	servDesc: A Simple example to discover services of nodes on the network.
-	stressTest: A test example used for testing the robustness of the framework.

The Platforms currently supported are:

The framework and examples are described in more detail in the UserGuide in the /docs directory of this git.

## Quick Start Guide

### Required HW
For the Linux platform only the [CC2538DK]( https://store.ti.com/CC2538DK-CC2538-Development-Kit-P4532.aspx) is required.

For the TIRTOS Tiva platform you will need in addition to the [CC2538DK]( https://store.ti.com/CC2538DK-CC2538-Development-Kit-P4532.aspx) ; a [TIVA-C Launchpad](https://store.ti.com/Tiva-C-LaunchPad.aspx) and an [EM Booster pack](https://store.ti.com/boost-ccemadapter.aspx ) (modified as indicated in the Users Guide).

### Programming the CC2538EM's with ZNP FW 
Use the [SmartRF Programmer 2]( http://processors.wiki.ti.com/index.php/Download_CCS) to program the FW contain in the bin dir of the git repository.

For the Linux Platform use /bin/cc2538-znp-120-usb-tclk.hex FW. For the TIRTOS/TIVA Platform use the cc2538-znp-120-uart-tclk.hex

Remember to set the P5 header correctly on the CC2538EM:

* For use with the Linux platform set P5 to VDD -> USB so the CC2538Em is powered from the USB connector.
* For programming and use with the TIRTOS/TIVA Platform set P5 to  VDD -> EM, so it is powered from the RM headers.

Connect the CC2538EM to the Host:

* For the Linux platform connect the CC2538EM to the Linux machine.
* For the TIRTOS/TIVA platform connect the CC2538EM to the EM Booster Pack and then to the TIRTOS/TIVA Launchpad.

###  Building and running the examples

The following describe how the build environment is setup and how one of the examples can be run. For a complete description consult the Users Guide.

Clone the git 

    git clone git://git.ti.com/znp-host-framework/znp-host-framework.git

#### Linux

Change dir to the build directory for one of the examples and build it

    cd znp-posix-framework/examples/cmdLine/build/gnu/
    make
    
Then run the example:

    ./cmdLine.bin /dev/ttyACM0
    

#### TI RTOS

Download CCS v6 from here:
[CCS v6]( http://processors.wiki.ti.com/index.php/Download_CCS)

Download TIRTOS for TIVA-C version 2.00.02.36 from here:
[TIRTOS for TIVA-C version 2.00.02.36 (for Windows)]( http://software-dl.ti.com/dsps/dsps_public_sw/sdo_sb/targetcontent/tirtos/2_00_02_36/exports/tirtos_tivac_setupwin32_2_00_02_36.exe)

Other versions of TIRTOS are available [here]( http://software-dl.ti.com/dsps/dsps_public_sw/sdo_sb/targetcontent/tirtos/index.html) however these are not yet tested so we recommend that you use the version suggested above.

Open CCSv6, Import the project through Project->Import->Code Composer Studio->CCS Project. Browse to <base dir>/znp-posix-framework\examples\cmdLine\build\tirtos\ccs

To build and download the example Right click on the example in the Project Explorer and select Debug As->Code Composer Debug Session

## Known Issues and Limitations

* When built from ZStack-Home-1.2.1 ZNP for CC2538 USB the host framework intermittently receives corrupted RPC messages. For this reason hex files from ZStack-Home-1.2.0 are included. The root cause of this issue is currently being investigated.

* API documentation is not yet released.

* stressTest Example with ED's results in high level of lost packets. This is because the Ed device sleeps and polls it parent at a default rate, its parent is responsible for buffering any messages until it polls. In this test the Coordinator will send a test message every 200ms, this can cause the parents buffer to overflow and messages to get lost. A possible resolution would be to configure the EndDevice a Always On for this test, but this has not yet been implemented. 
