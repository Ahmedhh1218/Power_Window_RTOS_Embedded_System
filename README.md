# Power_Window_RTOS_Embedded_System
This repository contains the code for an embedded real-time system that controls the front passenger door window with both passenger and driver control panels. The system's basic features include manual open/close, one-touch auto open/close, window lock, and jam protection functions.

Requirements
FreeRTOS
2 limit switches to limit the window motor from top and bottom limits of the window
Obstacle detection implementation using a push button to indicate jamming
System Features
Manual open/close function
When the power window switch is pushed or pulled continuously, the window opens or closes until the switch is released.

One touch auto open/close function
When the power window switch is pushed or pulled shortly, the window fully opens or closes.

Window lock function
When the window lock switch is turned on, the opening and closing of all windows except the driver's window is disabled.

Jam protection function
This function automatically stops the power window and moves it downward about 0.5 second if foreign matter gets caught in the window during one touch auto close operation.

Getting Started
Clone the repository and open the project in an IDE that supports FreeRTOS. Compile and upload the code to the microcontroller board.

Contributors
Team 2
