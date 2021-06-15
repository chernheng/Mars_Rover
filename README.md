# Mars_Rover
Submission repository of Group 22 for the Design Project for Year 2 EEE/EIE students from Imperial.

## Instructions
- For Drive and Energy, open the Arduino code in the relevant folders, compile, and upload to the Arduino Nano Every.
- For Control, alter the source code to add in Wifi SSID and password, and upload to the ESP-32.
- For Vision, open up the Quartus `.qpf` file in Quartus 16.1, and use the Programmer to blast the `.sof` file in `Vision/output_files` into the FPGA. Subsequently, import `Vision/software/D8M_Camera_Test` and `Vision/software/D8M_Camera_Test_bsp` into the Nios II SBT Eclipse workspace, and Run `main.c` as Nios II hardware.
- For Command, @Xinyue

## Troubleshooting
If code for the Drive or Control subsystems does not upload to their respective microcontrollers, try to plug them out from their respective carrier boards before flashing the code. Subsequently, plug them back into their carrier boards, and setup the FPGA.

## Sources
We acknowlege the sample code provided: for Drive, Yue Zhu, for Energy, Dr Phil Clemow, for Vision, Dr Ed Stott. In addition, demo code for the D8M-GPIO camera from Terasic was used.