# Overview

This repository is used to maintain and reflash the ESCs on rovers when necessary. To use it, you will need to first download platformIO extension in VSCode (as seen below):
<img width="951" alt="platformIO-extension" src="https://github.com/user-attachments/assets/cea15b31-50ab-4489-95ce-837e37c3c3e7" />

 Next, you will need to rebuild the project and flash it to the ESC using the gui integrated into VSCode. To build it, you need to click on the checkmark icon in the bottom left corner of the screen:

<img width="502" alt="build" src="https://github.com/user-attachments/assets/56f05e3d-334c-40a1-a9b8-5de7c5bb0567" />

Now, you can connect the ESC to one of the USB ports on your laptop and find an arrow icon to flash it:

<img width="500" alt="flash" src="https://github.com/user-attachments/assets/a6baa441-6532-460d-abba-b27d1117ecab" />


## Rationale
We attempted to set this up in a devcontainer, but unfortunately that required mounting the usb interface to the devcontainer. This, in turn, gave rise to several issues on MacOS, where a cumbersome workaround would be necessary to get it to work. 