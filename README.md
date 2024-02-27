### To run the code on a microcontroller
Upload the code using **PlatformIO** extension of **Visual Studio Code**.


### To run the python script
First install the required packages:
`pip3 install bleak`
`pip3 install pandas`
then navigate to the **pc-remote** folder and run the script:
`python3 communicateViaBLE.py`
The python script should connect to the board automatically.
Use terminal to send predefined commands (send **help** for full command list preview).


### To enable manual control using a transmitter
Connect to PC a transmitter via USB or wirelessly via an RF siumaltor dongle (e.g., *FrSky XSR-SIM*).
Then install the **PySticks** package by navigating to its directory and runing the script:
`python3 communicateViaBLE.py`
You should now modify the joystick axes to send desired commands and switch to **manual** control mode.


### Microcontroller info:
- **Board**: from Seeed Studio -> XIAO BLE nRF52840 Sense
- **Programmer**: based on *Adafruit FreeRTOS* core implementation (not the *mbed-enabled* core)
- **Framework**: based on Arduino codebase
- **Code Development**: VSCode -> PlatformIO