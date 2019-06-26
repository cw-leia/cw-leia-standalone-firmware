LEIA Firmware with Wookey HAL
=============================

Introduction
------------
This project contains the source code for building the firmware of LEIA (Lab Embedded ISO7816 Analyzer) smartcard reader.

More on [LEIA: the Lab Embedded ISO7816 Analyzer A Custom Smartcard Reader for the ChipWhisperer](https://www.sstic.org/2019/presentation/LEIA_the_lab_embedded_iso7816_analyzer).

Compilation
-----------

To compile the project, you will need a gcc compiler for arm.

Provided functionalities are the following:
   * ISO7816 stack
   * Triggers configuration
   * Communication via UART 

To compile the firmware:

~~~
make firmware 
~~~

To flash LEIA:

~~~
make burn
~~~
