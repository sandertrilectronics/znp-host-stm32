# znp-host-stm32
An example project for interfacing with the ZNP firmware on a CC2530 Zigbee Module from TI using an STM32 microcontroller.

This project expands the znp-host-framework as found in:

https://github.com/pengphei/znp-host-framework

The framework has been ported to the STM32 and some extra missing commands are added. A lot of other code that was not meant for an embedde device was removed (file IO, malloc, etc). The goal of this repository is to support all commands in the ZNP firmware, just as Z-Tool on the PC would, but executable from the STM32 processor (or any other embedded device).

# Software Buildup
The project uses FreeRTOS to manage two tasks: 
- The application task and;
- The communication task.

Beside these two tasks there are four other OS objects:
- Two queues for the sending and receiving of data bytes from the UART;
- One queue for the transfer of data frames that are received;
- One semaphore for reception frame signaling.

A normal command cycle will look like the following:

```
1. APP TX: 
    APP TASK -> UART TX Queue -> CC2530  

2. Wait RX: 
    APP TASK Waits For Synchronous Response semaphore (srspSem)

3. Data RX: 
    data frame queue <- COM TASK <- UART RX Queue <- CC2530

4. Done RX: 
    COM TASK Gives Synchronous Response semaphore (srspSem)

5. APP RX: 
    APP TASK <- data frame queue
```

The application task contains some examples for interfacing, but a lot of examples that can be found for the Z-Tool GUI can be easily ported to this code.


# Building
The project is build using STM32Cube IDE. This IDE is supplied by ST Microelectronics free of charge for all of it's microcontrollers. It can be downloaded from:

https://www.st.com/en/development-tools/stm32cubeide.html

In the IDE the project can be imported by selecting:  

```file -> Open Projects From Filesystem...```  

and browsing to the root directory of the project.

# Hardware
The used hardware for this project is the P-L496G-CELL02 (without BG96 modem) from ST and the CC2530 Eval Kit from Waveshare.

https://www.st.com/en/evaluation-tools/p-l496g-cell02.html

https://www.waveshare.com/cc2530-eval-kit.htm

The CC2530 Eval Kit runs the ZNP firmware found in Z-Stack 3.0.2. For convienence it is also found in this repository as source file (CC2530ZNP-with-SBL.hex). This should be flashed on the CC2530 using the CC-Debuger from TI before connecting it with the STM32.

Some connections between the modules are to be made. The Arduino headers on the bottom side of the P-L496G-CELL02 are used to connect to the CC2530 Eval Kit. The connections are as follows:

```
ST  -> CC2530  
GND -> GND  
3V3 -> VCC  
D0  -> P0_3  
D1  -> P0_2  
```

On the CC2530 Eval Kit the onboard USB converter should be disconnected. This can be done by removing the yellow headers which are placed on the "UART0 JMP" jumper.

# TODO
- The framework can still be expanded by adding more commands.
- The tasks and interfaces are not nicely coded at this point.
- Sending data is not yet thread safe.
