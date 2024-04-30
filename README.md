# AVC
## Hardware
### Intel RealSense Depth Camera
In order to run this code, you will need an Intel RealSense Depth Camera. 

To use this camera, you will need to download the Intel RealSense SDK, found here: https://www.intelrealsense.com/sdk-2/

To use this camera with our program, you will need to install Pyrealsense using this command: ` pip install pyrealsense2 `

### UART Communication 
In order to run this code, you will need a way to transmit UART commands. For our implementation, the UART communication was wired from the Electrical Team's controller board to the connector pins on the Jetson Nano.  

Without a way to simulate UART commands, the program will send an error code looking for a connection to the /dev/ttyTHS1 port. 

## Running the Program 
Once all the hardware is set up, navigate to the directory in which it is downloaded and run this command: ` python3 avc.py ` 

Please note that the realsense_depth.py code is initializing the camera and needs to be in the same directory as avc.py for everything to work properly. 
