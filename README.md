#### To run the code on a microcontroller
Upload the code using **PlatformIO** extension of **Visual Studio Code**.


#### To run the python script
First install required packages:
`pip install bleak`
`pip install pandas`
then navigate to the **pc-remote** folder and run the script:
`python communicateViaBLE.py`
The python script should connect to the board automatically.
Use terminal to send predefined commands (send **help** for full command list preview).


#### Microcontroller info:
- **Board**: from Seeed Studio -> XIAO BLE nRF52840 Sense
- **Programmer**: based on *Adafruit FreeRTOS* core implementation (not the *mbed-enabled* core)
- **Framework**: based on Arduino codebase
- **Code Development**: VSCode -> PlatformIO