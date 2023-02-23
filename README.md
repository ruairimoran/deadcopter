# Deadcopter is alive!*

*on Python 3.8

Stay tuned.


## Arduino set up

For aurdino to be able to access the correct libraries from this repository you will have to change your sketchbook location to the arduino folder cloned from this repository on your local device.

To do this go to **File > Preferences** and change  your sketchbook location on the arduino IDE.


## How to fly

### Quick start
Upload due_v2.ino onto arduino board

Open terminal on Raspberry Pi and execute the following 
```
sudo pigpiod
```
```
python deadcopter/Pi/read_sbus/read_sbus_from_GPIO_receiver.py
```
<sup> ^^ Make sure file path is correct! </sup>

Ensure Arm and Kill switch are off --- *(for current setup Arm is switch B, and Kill is switch A, switch in up postion is off and down position on, switches located top left of RadioLink AT10II)*

Turn on transmitter and Arm --- *(propellers will spin for a second)*

**Quadrotor is ready to fly!**

***Please fly responsably***


## Simulations

### Quick start
Open terminal and execute the following 
```
python deadcopter/sim.py
```
<sup> ^^ Make sure file path is correct! </sup>

### Making Changes
#### LQR Q and R gains
To make changes to the LQR Q and R gains go to lines **107-108** in the `coptor.py` file which can be found under `deadcoptor/src/flight_controller`

#### Plots
To change between orientation graph only and all graphs, go to line **112** in the `simulator.py` file which can be found under `deadcoptor/src/flight_controller`

### Issues
If simulations aren't working try checking the import path in the `sim.py` and `__init__.py` file. (I had to remove deadcoptor. from start)


## Updating LQR gain K 
After simulating and determining LQR weights for Q and R, a new gain matrix K needs to be calculated and updated on the Arduino code. To do this go to lines **88-89** in the `generate_new_mats.py` file, currently found under `deadcoptor/arduino/due_v2`, *(will probably be moved to src folder)* and input the new Q and R values.

Open terminal and execute the following 
```
python deadcopter/arduino/due_v2/generate_new_mats.py
```
<sup> ^^ Make sure file path is correct! </sup>

This will calculate the new LQR gain K and update the `ss_and_mats.h` file where this is used in the Arduino code.


## Notes

- `src`: main Python package
- `src/test/test_*.py`: unit tests
- `sim.py`: runnable Python file for simulator
