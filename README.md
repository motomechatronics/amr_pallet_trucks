# amr_pallet_trucks project
## Table of Contents
- [1. Overview](#1-overview)
- [2. Installation](#2-installation)
- [3. Usage](#3-usage)

### 1. Overview

The four industrial revolution (4IR) pushes the manufacturing industry to an integration with newest technologies like the Artificial Intelligence, additive manufacturing, augmented reality and the IoT internet of things. The 4IR also includes an advanced in connectivity between machines, robots, sensors, etc. that gives rise to “smart factories”.

The following project refers to a smart factory in which the IoT, the advanced connectivity and the ROS technology allow an autonomous management of the raw and waste materials. In particular, the environment, shown in the figure 1.1, reproduces a mechanical workshop for working metals with numerical control machines, where an AMR pallet truck ensures the handling of both raw or semi-finished pieces and the transport of residual metal shaving.

<p align="center">
<img src="docs/mechanical_workshop.png" width="700" />
</p>

<h4 align="center">
Figure 1.1: Mechanical workshop environment. (1) AMR pallet trucks, (2) CNC machine, (3) full chip bin, (4) empty chip bin.
</h4>

In the following scenario, it is staged an example of management of the residual metal chip bin by the AMR pallet truck using a web interface, as shown in figure 1.2. It leaves the fully automatic management to a later implementation.

<p align="center">
<img src="docs/webinterface.png" width="700" />
</p>

<h4 align="center">
Figure 1.2: Web Interface. Buttons: (1) go to full bin, (2) fork the bin, (3) elevator up, (4) go to recycling, (5) elevator down, (6) disengagement bin, (7) recycling bin, (8) go to empty bin, (9) go to CNC machine, (10) go home, (11) move forward, (12) move backward, (13) move turn CW, (14) move turn CCW, (15) stop, (16) connect/disconnect. Readings: (17) odometry position, (18) obstacle distances.
</h4>

The project, created using ROS Noetic and Python language, has been divided into four main parts:
 - Gazebo simulation 
 - chip bin management
 - navigation 
 - usage

The first part concerns the construction of the AMR and its environment, i.e. the mechanical workshop with CNC machines and the chip bins. The second one is dealing with the actions of the AMR, like: the detection, the engagement and the disengagement of the chip bin. The third regards the transport of both empty and full europallets. The last one is an explanation on how to do a chip bin replacement cycle using a web interface. To achive these objectives  9 packages were created, grouped as follows:

 - **Gazebo simulation** 
     - amr_description
     - amr_environment     
 - **chip bin management**
      - amr_detection
      - amr_rendezvous
      - amr_disengagement  
      - amr_webpage
 - **navigation**
      - amr_mapping
      - amr_localization
      - amr_navigation
 - **usage**
 
[go to top](#amr_pallet_trucks-project)

### 2. Installation
Open a terminal and type

```
cd ~/catkin_ws/src
git clone https://github.com/motomechatronics/amr_pallet_trucks.git
cd ..
catkin_make
source devel/setup.bash
export NVM_DIR="/home/user/catkin_ws/src/amr_pallet_trucks/webpage_ws/nvm"

```
Next, always in the same terminal, use **vim**, **gedit**, or ...
```
vim ~/.bashrc
```
At the end of the file, (press i) copy and paste:
```
export NVM_DIR="/home/user/catkin_ws/src/amr_pallet_trucks/amr_webpage/nvm"
[ -s "$NVM_DIR/nvm.sh" ] && \. "$NVM_DIR/nvm.sh"

```
press esc and save typing inside **vim** environment
```
:wq!
```
In the same terminal
```
source ~/.bashrc
nvm ls
nvm install v14
nvm alias default v14
npm install -g http-server
 ```
 
[go to top](#amr_pallet_trucks-project)
### 3. Usage
Open a terminal and type
```
roslaunch amr_description main_project.launch
```
Open another terminal and type
```
cd catkin_ws/src/amr_pallet_trucks/amr_webpage/
http-server --port 7000
```
Open another terminal and type
```
webpage_address
```
You will get a web link. Copy and paste the link in a browser or clink on the link. After in the same terminal type
```
rosbridge_address
```
You will get an alphanumeric string, copy and paste the string into the web interface shown in figure 3.1 and press the **connect** button.

<p align="center">
<img src="docs/webinterface.png" width="700" />
</p>

<h4 align="center">
Figure 3.1: Web Interface. Buttons: (1) go to full bin, (2) fork the bin, (3) elevator up, (4) go to recycling, (5) elevator down, (6) disengagement bin, (7) recycling bin, (8) go to empty bin, (9) go to CNC machine, (10) go home, (11) move forward, (12) move backward, (13) move turn CW, (14) move turn CCW, (15) stop, (16) connect/disconnect. Readings: (17) odometry position, (18) obstacle distances.
</h4>

At the moment the project manages only one bin, not selectable. The procedure to stage a substitution cycle is the following:
- press **go to full bin** button, the expected result is shown in figure 3.2.

<p align="center">
<img src="docs/approch_full_bin2.gif" width="700" />
</p>

<h4 align="center">
Figure 3.2: result pressing the go to full bin button.
</h4>

after that, 
- press **fork the bin** button and next the **elevator up** button, the expected result is shown in figure 3.3.

<p align="center">
<img src="docs/fork and lift full bin.gif" width="700" />
</p>

<h4 align="center">
Figure 3.3: result pressing the fork the bin and the elevator up buttons.
</h4>

- press **go to recycling** button and next the **elevator down** button, the expected result is shown in figure 3.4.

<p align="center">
<img src="docs/move to recycling area lift down full bin.gif" width="700" />
</p>

<h4 align="center">
Figure 3.4: result pressing the go to recycling and elevator down buttons.
</h4>

- press **disengagement** button and when the AMR will be disengaged press **recycling bin** and **go to empty bin** buttons. The expected result is shown in figure 3.5.

<p align="center">
<img src="docs/disengagement and go to load empty bin.gif" width="700" />
</p>

<h4 align="center">
Figure 3.5: result pressing disengagement bin, recycling bin and go to empty bin buttons.
</h4>

- pressing **fork the bin** button, the expected result is shown in figure 3.6.

<p align="center">
<img src="docs/fork empty bin and lift up.gif" width="700" />
</p>

<h4 align="center">
Figure 3.6: result pressing the go to empty bin button.
</h4>

- pressing **go to cnc machine** button and next the **elevator down** button, the expected result is shown in figure 3.7.

<p align="center">
<img src="docs/go to cnc machine and lift down empty bin.gif" width="700" />
</p>

<h4 align="center">
Figure 3.7: result pressing the go to cnc machine and elevator down buttons.
</h4>

- pressing **disengagement** and **go home** buttons the cycle ends. The expected result is shown in figure 3.8.

<p align="center">
<img src="docs/disengagement empty bin and go home.gif" width="700" />
</p>

<h4 align="center">
Figure 3.8: result pressing disengagement and go home buttons.
</h4>
[go to top](#amr_pallet_trucks-project)
