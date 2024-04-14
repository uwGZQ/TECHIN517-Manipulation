# TECHIN517-Manipulation

## Instructions for Camera Setup D435i
refer to this link for installing dependencies, librealsense2 and the build. 
https://dev.intelrealsense.com/docs/compiling-librealsense-for-linux-ubuntu-guide?_ga=2.42442656.40329647.1712875214-856519986.1712454558

>Everything is pretty much covered , check if the patching is done correctly by running this command:

`sudo dmesg | tail -n 50` \
The log should indicate that a new _uvcvideo_ driver has been registered.  
Refer to [Troubleshooting](#troubleshooting-installation-and-patch-related-issues) in case of errors/warning reports.

>check the camera by running : 
`realsense-viewer` 


