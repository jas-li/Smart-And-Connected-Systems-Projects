# Mission Impossible

Authors: Jake Lee, Jason Li,  Maxim Slobodchikov, JiaLin Sui

Date: 2023-09-22

### Summary
Our goal was to create a Mission Impossible inspired alarm system
Our device is designed to sense movement, increase in room temperature, and floor contact


### Self-Assessment 

| Objective Criterion | Rating | Max Value  | 
|---------------------------------------------|:-----------:|:---------:|
| 1. Reports time to console along with sensor readings  | 1 |  1     | 
| 2. Measures input from photocell | 1 |  1     | 
| 3. Measures and reports temperature in Engineering units  | 1 |  1     | 
| 4. Cyclic behavior at design frequency  | 1 |  1     | 
| 5. Uses hardware interrupt for button press   | 1 |  1     | 
| 6. Lights up correct LEDs on alarms    | 1 |  1     | 


### Solution Design

Floor contact
- We employed a button that uses hardware interrupts triggered by a button press. This simulates someone triggering a pressure sensor on the floor of the room. 
- When the button is pressed, a flag is triggered and our system lights up the blue LED to indicate that someone is standing in the room
  
Movement
- simulates movement by measuring changes in ambient lighting using a photocell
- The photocell sensor is connected to one the esp32’s adc channels
- We convert the adc values into voltage in order to observe the intensity of light
- When the photocell senses that the light intensity is below the average intensity from the overhead light, it triggers our system to light up a yellow LED to indicate movement

  
Temperature Change
- Sense an increase in the room’s temperature from the body heat of a human
- Our program checks for when the room temperature reaches our threshold of 21 degrees Celsius. Once the temperature reaches this temperature, the Red LED is lit up to indicate the change in temperature
- Note: Steinhart-Hart equation takes the resistance of the thermistor to convert it into a temperature unit
  
Cyclic Design
- All the devices were implemented as a periodic cycling of every 0.5 seconds evaluation of the sensors, specifically temperature and light followed by reporting of current values and setting of the output LEDs and we used a hardware interrupt for the button press
- When everything is all clear, a green led is lit up



### Sketches/Diagrams
<p align="center">
<img src="images/Screenshot 2023-09-22 at 3.23.18 PM.png" width="50%">
</p>
<p align="center">
<i>Sample Terminal Output</i>
</p>

<p align="center">
<img src="https://github.com/BU-EC444/Team7-Lee-Li-Slobodchikov-Sui/assets/114517092/4dd08309-7938-4756-b913-5bef0a4c8636" width="50%">
</p>
<p align="center">
<i>Circuit Diagram</i>
</p>

<p align="center">
<img src="images/EC444 Quest 1.png" width="50%">
</p>
<p align="center">
<i>Code Flow Chart</i>
</p>



### Supporting Artifacts
[![Technical Video](<./images/Screenshot 2023-09-22 at 3.51.22 PM.png>)](https://drive.google.com/file/d/1AKb4MOBwkUd8NBCEyzgFDFEzxutGBv41/view?usp=drive_link)
<p align="center">
<i>Technical Video</i>
</p>

[![Demo Video](<./images/Screenshot 2023-09-22 at 3.51.00 PM.png>)](https://drive.google.com/file/d/1OT7nLqu3c-im51rbB84WRpaeCsU0cNX_/view?usp=drive_link)
<p align="center">
<i>Demo Video</i>
</p>


### Modules, Tools, Source Used Including Attribution
https://www.e-tinkers.com/2019/10/using-a-thermistor-with-arduino-and-unexpected-esp32-adc-non-linearity/

https://github.com/BU-EC444/04-Code-Examples/blob/main/button-timer/main/main_1.c


