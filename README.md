# ms26
Mini Sumo DIY for Spring 2026 RoboRama.<br>
This is a DIY mini sumo 10cmx10cm robot made with "off the shelf" modules and hand wired 0.1" bread board circuits.<br>

# Mechanics
## Body
The body is in 2 parts priinted on 3D printer, a main body and a top lid and is created using Matte Black PLA filiment.<br>
The main body is one piece and has mounts for the motors/wheels and the circuit board and sensors. There is an opening on the side for the battery to be inserted.<br>
The top lid bolts onto the body.<br>
3D design using FreeCad:<br>
<img src="support/mainBody3D.jpg">
The final mini-sumo robot without the lid<br>
<img src="support/miniSumo-noLid.jpg">
The object and edge sensors are mounted as well as wheels and motors. The motors are bolted into place using a 3D saddle mount<br>
<img src="support/ms_noCircuitBoardYet.jpg">

## Weight
The weight limit is 500 grams. Without a lid it weighs in at about 225 grams.
<img src="support/ms_weight.jpg">

## Motor
FingerTech "Silver Spark" 16mm Gearmotor 50:1<br>
<https://www.fingertechrobotics.com/proddetail.php?prod=ft-Sspark16><br>

## Wheel

FingerTech Mini-Sumo Wheels (pair)<br>
<https://www.fingertechrobotics.com/proddetail.php?prod=ft-minisumo-wheels-1125><br>
The wheels and motors are made to be used together for the 10cm width on mini-sumo.<br>
<img src="support/WheelSpacing.jpg"><br>


## Battery

# Circuit modules
## Processor
## 6V regulator
This supplies power for the processor and 3.3V regulator. The 6V output allows a protection diode while keeping the voltage above 5V</br>
Pololu 6V, 2.7A Step-Down Voltage Regulator D36V28F6</br>
<https://www.pololu.com/product/3783>

## 3.3V regulator
This supplies 3.3V power to the sensors. The 3.3v from the RP2040 module does not have the current rating for the sensors.<br>
Pololu 3.3V, 600mA Step-Down Voltage Regulator D36V6F3.<br>
<https://www.pololu.com/product/3791><br>

## Motor driver
## Range sensor
## Edge detector
## Object detector
## IMU

# Circuits
The 
<img src="support/ms26_schematic.jpg">



