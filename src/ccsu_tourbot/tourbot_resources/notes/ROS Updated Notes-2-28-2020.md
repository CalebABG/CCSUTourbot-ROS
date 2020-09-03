# Notes for ROS


## Updates to do - 2/28/2020
1. Update Lidar Firmware
2. Update Firmware of IMU (Razor M0 9Dof) [```ROS Razor_AHRS.ino```]
   1. Edit file and uncomment #define for HW version for Razor M0


---

## Notes For IMU and RPLidar

### RPLidar Info
- Model: A2M8
- Baud Rate: 115200
- Hardware Version: 5
- Serial Number: B0F19AF2C1EA98D4BEEB9CF03F223517
- Firmware Version: 1.27


### Razor M0 IMU
- Accel Calibration: #oc
  - X:
    - -256.96/254.64
  - Y:
    - -252.81/255.74
  - Z:
    - 70.68/148.32
  
- Gyro Calibration: #on (move to gyro), #oc (reset)
  - Average: 
    - -0.17/-0.17  0.02/0.02  0.32/0.32



---



## **Fresh ROS Install**
1. **Navigate to Ubuntu ROS install** [ROS Kinetic Install](http://wiki.ros.org/kinetic/Installation/Ubuntu)
   1. Follow install instructions:
      1. **READ INSTRUCTIONS CAREFULLY**, some commands have multiple additional terminal commands, but usually there's only one **MAIN** command to run
      2. When at 'Installation step' choose ```Desktop-Full Install```

2. **Installing** ```catkin_tools``` [Link](https://catkin-tools.readthedocs.io/en/latest/installing.html)
   - In terminal execute commands:
      1. ```sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" > /etc/apt/sources.list.d/ros-latest.list'```
      2. ```wget http://packages.ros.org/ros.key -O - | sudo apt-key add -```
      3. ```sudo apt-get update```
      4. ```sudo apt-get install python-catkin-tools```

3. **Installing** ```ROS Open-Gmapping package```
   - In terminal execute commands:
      2. ```sudo apt-get install ros-kinetic-openslam-gmapping```

4. **Installing** ```Python Pip```
   - In terminal execute commands:
     - ```sudo apt-get install python-pip``` # Ubuntu comes with Python 2.7, but most likely does not have Pip installed with it

5. **Installing** ```Pyserial``` and ```Easyprocess```
   - In terminal execute commands:
     - ```pip install pyserial```
     - ```pip install easyprocess```


6. **Initializing a New Workspace**
   - In terminal execute commands:
      1. ```mkdir -p ~/ros_ws/src``` # Make a new workspace and source space
      2. ```cd ~/ros_ws/``` # Navigate to the workspace root
      3. ```catkin_make``` # Build the new workspace
      4. ```source ~/ros_ws/devel/setup.bash``` # Source the new build w/ setup script


---


## Setup of Existing ROS Workspace (i.e Moving/Copying ros_ws to new Ubuntu install)
**Reason to use ```Copy To``` action**
- For some reason, regular ```copy``` and ```paste``` actions make it a nightmare when trying to build (use ```catkin_make``` command)
  - Possibly because direct ```copy``` and ```paste``` actions maintain file permissions. Either way, just use ```Copy To``` :D

1. When needing to copy files or folders from **source** (i.e possibly a ```USB drive```), to a **destination** (i.e the new ```ros_ws```); use the ```Copy To``` action.
   - Right-click on the file or folder, and select the ```Copy To``` action
   - Then navigate to the **destination** where you want the file/folder to be copied to; then hit **select** to finish


---


## Pycharm IDE and Visual Studio Code editor install and setup (Pycharm is optional, it's a full-fledged editor)
### **Installing** ```Visual Studio Code```
1. Download the ```.deb``` file and open it (should open ```Ubuntu Software``` application to install)
2. Install it through the application (much easier and quicker)
3. Once installed, now add useful and handy ```extensions```
- In terminal execute command:
 - 
   ```
   cat ~/ros_ws/src/ccsu_tourbot/tourbot_resources/notes/ros_vscode_extensions.txt | xargs -L 1 code --install-extension
   ```

#### **Export VS Code Installed Extensions**
- In terminal execute command:
  - 
   ```
   code --list-extensions > ~/ros_ws/src/ccsu_tourbot/tourbot_resources/notes/ros_vscode_extensions.txt
   ```

<br/>

### **Installing** ```Pycharm IDE```
1. Download the ```.tar.gz``` file (Download the ```Community``` edition)
2. Click and open the downloaded file in ```Archive Manager``` application
3. Drag the only folder in ```Archive Manager``` to ```Home``` folder on Hard-drive
4. Open the newly extracted folder ```pycharm-community-...``` ('...' will be a version number)
5. Right-click inside the folder and select the ```Open in Terminal``` action
6. In the terminal execute the following commands:
   1. ```cd bin/```
   2. ```./pycharm.sh```
7. Once the program start, follow setup instructions
8. When setup is finished, you should be at this screen: ![](pycharm_setup_mainscreen.png?raw=true "Pycharm IDE Mainscreen")
9. On this screen, click the ```Configure``` option
   1.  Then click ```Create Desktop Entry```
   2.  When the popup appears, check the 'Create entry for all users'

<br/>


#### Installing ROS ```Hatchery``` Plugin
1. Open ```Pycharm IDE```
2. When at Main screen (refer to picture above), click the ```Configure``` option
   1. Then click the ```Plugins``` option
   2. In the search window, type: ```Hatchery```
   3. Install the plugin from: 'Duckietown'
   4. Restart IDE


---


## **Other Tools and Helpful Stuff** (Installs are optional)

### ROS Lecture Notes/Cheat sheets/(Issue + Resolve) Links

- [ROS Cheat-sheet](https://github.com/ros/cheatsheet/releases/download/0.0.1/ROScheatsheet_catkin.pdf)

- [ROS Best Practices](https://github.com/leggedrobotics/ros_best_practices/wiki)

- [IMU Bias Calibration/Bias Comp Package](https://github.com/ros-perception/imu_pipeline)

- [ROS Rqt_tf_tree Fix](https://answers.ros.org/question/132946/rqt_graph-not-shown-in-window/)

- [Python Linter Fix](https://github.com/pypa/pip/issues/524)

- [Python Pip uninstall Fix](https://github.com/pypa/pip/issues/3776)


<br/>


### **Chromium Browser**
1. Open ```Ubuntu Software``` application
2. In search bar, type: 'Chromium'
3. Install the ```Chromium``` application from **Developer:** ```Canonical```


<br/>


### **Ubuntu Terminator Terminal**
- In terminal execute command:
  - ```sudo apt-get install terminator```


### **Ubuntu SmartGit Git GUI**
1. Navigate to [SmartGit Download](https://www.syntevo.com/smartgit/download/)
2. Download .deb (```Debian Bundle```)
3. Click to open file, should open in ```Ubuntu Software``` application
4. Install through ```Ubuntu Software``` (easier and quicker)
5. Once installed, open SmartGit application
6. Follow instructions for setup


<br/>


### **Linux ```chmod``` for a folder and all of its sub-folders and files**: [Link](https://stackoverflow.com/questions/3740152/how-do-i-set-chmod-for-a-folder-and-all-of-its-subfolders-and-files)

- In terminal execute commands:
  - ```sudo chmod -R 755 /directory``` # **REPLACE** '/directory' with the actual directory you want


<br/>


### **ROS Joystick Package**
- In terminal execute command: ```sudo apt-get install ros-kinetic-joy```


<br/>


### **Ubuntu Event info Package**
- In terminal execute command: ```sudo apt-get install evtest```


<br/>


### **Testing connected interfaces**
- In terminal execute command: 
  1. ```lspci```
  2. ```lsusb```
  3. ```cat /proc/bus/input/devices```


<br/>


### **Pairing Bluetooth devices using bluetoothctl**
1. Make pc discoverable (just in case need be for device)
2. Hold x + guide button on controller to set it to pairing mode
3. Select gamepad to pair and hit next and wait till it finishes configuring

- In terminal execute commands: 
  1. ```bluetoothctl```
  2. ```devices``` # Copy the MAC for the gamepad
  3. ```trust 'MAC'``` # **REPLACE** the 'MAC' with the **MAC Address** for the gamepad
  4. ```exit``` # Exit bluetoothctl


#### **Disabling controller Mouse pointer control**
- In terminal execute commands:
  1. ```xinput list```
  2. Find the ```*id*``` of the gamepad and copy it
  3. ```xinput disable 'id'``` # **REPLACE** 'id' with the id of the gamepad to stop the gamepad from controlling the mouse 

#### You can re-enable the input again by: xinput enable 'id' (where 'id' is the id of the gamepad)


<br/>


### **ROS rf2o_laser_odometry Package**
#### Mobile Robot Programming Toolkit MRPT
- In terminal execute commands:
   1. ```sudo add-apt-repository ppa:joseluisblancoc/mrpt```
   2. ```sudo apt-get update```
   3. ```sudo apt-get install libmrpt-dev mrpt-apps```


<br/>


### **ROS LSD Slam Package**
- In terminal execute commands:

   #### Video Driver Tools
   sudo apt-get install v4l-utils

   #### Dependencies for lsd_slam Package
   ```
   sudo apt-get install ros-kinetic-libg2o ros-kinetic-cv-bridge liblapack-dev libblas-dev freeglut3-dev libqglviewer-dev libsuitesparse-dev libx11-dev
   ```

<br/>


### **ROS Scan Tools Package**
- In terminal execute commands:
  - ```sudo apt-get install ros-kinetic-scan-tools```