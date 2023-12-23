# Carmin Watch
Authors: Jason Li, Jake Lee, Maxim Slobodchikov, Jialin Sui

Date: 10/08/2023

# Code Overview
For this Quest we leveraged RTOS tasking and Node.js in order to organize the different functions for our Carmin watch and actively track our sensor data onto an HTML server. 

## Button Start/Stop/Reset
For our button, we implemented three states stored in an integer variable. State 0 - Reset, State 1 - Start Activity, State 2 - Stop Activity. We used button interrupts to trigger a flag in our esp32 code, which would change the state of our button. Depending on our state it, we would handle each differently. If the button was in a reset state, we would write to the serial port to our node.js server to clear/reset our data in our csv file and reload our web server. If the button was in the start state, we predefined a sampling interval in this case, 10 seconds, and every 10 seconds our sensor data will be written to the serial port for the node.js server to handle. In our stop state, no sensor data will be written to the serial port.

## Temperature monitor
For the temperature sensing aspect of this Quest, we used a thermistor probe. The probe is connected to one of the ADC channels on the esp32 board. Our code first receives the raw ADC values and converts them into a voltage input. This voltage input is then also used to calculate the resistance of the temperature sensor. Using this resistance we were able to apply Steinhart-Hart's equation to get the body temperature of the user. We set a threshold at 40 degrees Celcius (temperature of body overheating) before our buzzer is triggered. The temperature is sampled every second.

## Accelerometer/Step Counting
For our accelerometer sensor, we sampled data using I2C signals. We processed the sampled acceleration data every second and took the absolute value of the difference of the last reading and the current reading. This acceleration difference tells if a step was taken. If the difference was greater than 0.5 we know a step was taken and if it was less then it was just from noise. The steps are counted every 10 seconds, so we when we detected a step, we incremented a step counter and at the 10 second interval, the step count is written to the node.js server and count variable is reset. 

## Alphanumeric Display
For our alphanumeric display, we communicated to the display using I2C. We also used the node.js server to tell the esp32 the current time of day. The node.js server continuously writes to the serial port for the esp32 to read from every 5 seconds updating the time of day. The esp32 handles this time input and writes it to the alphanumeric display. 

## Node.js/Canvas
The nodeJS serves as our server where we graph and update sensor data in real-time. The nodeJS file first reads sensor data and (as a new line) writes it to the ``.csv`` file and then it uses ``fswatch()`` to watch for any changes to the csv file, when a change is detected it reads the latest line and pushes the data to the server which is used by the canvasJS to generate and update the graphs.

