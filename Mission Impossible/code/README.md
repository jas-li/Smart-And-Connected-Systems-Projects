# Mission Impossible
Authors: Jason Li, Jake Lee, Maxim Slobodchikov, Jialin Sui

Date: 09/22/2023

# Code Overview
For this Quest we leveraged RTOS tasking in order to organize and assign priorities to the various functions we want for our Mission Impossible Alarm task. 

## Button Task
For the first task, we implemented a button trigger to resemble floor contact. We programmed an interrupt for the button press. When the button is pressed, the flag is triggered and handled by our button control. It changes the state of the blue LED to be on and after a 1s delay the flag returns to 0.

## Thermistor Task
For the temperature sensing aspect of this Quest, we used a thermistor probe. The probe is connected to one of the ADC channels on the esp32 board. Our code first receives the raw ADC values and converts them into a voltage input. This voltage input is then also used to calculate the resistance of the temperature sensor. Using this resistance we were able to apply Steinhart-Hart's equation to get the temperature of the room. We set a threshold at 21 degrees Celcius before our alarm triggers. When the temperature reaches above this threshold, we change the state of our red LED to be on. The sensor samples every 0.5 seconds and prints the temperature onto the console.

## Photocell Task
For the light sensing part of the Quest, we used a photocell sensor connected to another ADC channel on the esp32. The conversion from ADC to voltage is the same as the thermistor task. We also set another threshold for the photocell. We first measured the average ambient lighting intensity, then we set our threshold to a value just below the average. If the intensity of light goes under the threshold, we change the state of the yellow LED to be on. This sensor also samples every 0.5 seconds and prints the voltage from the photocell onto the console.

## LED Control Task
For the LED controls, we received the LED states that came from the photocell, button, and thermistor tasks. From these tasks we passed the states through some conditionals statements to determine whether or not to set the levels of the LEDs to be on or not. This task would also sample every 0.5 seconds to match the change from the sensors. We also printed a time counter to let the user know how much time has elapsed that prints also every 0.5 seconds.
