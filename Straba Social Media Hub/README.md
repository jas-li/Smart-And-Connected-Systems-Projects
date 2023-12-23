# Straba Social Media Hub

Authors: Jake Lee, Jason Li , JiaLin Sui, Maxim Slobodchikov

Date: 2023-10-26

### Summary
The Straba Media Hub is portal for hosting data originating from multiple Carmins. The project iterated from our previous Carmin watch by collecting data from many individual smart watches, sourcing and passing data across a wireless network/routers in a bidirectional way, making data available from any browser, and aggregating data and displaying leaders for specific metrics.

The key features involved:
- Carmins connecting via WiFi
- Collecting data from multiple Carmins to central server and presenting data as web portal
- Central server reports leader status back to each Carmin alpha display
- Central server runs on node.js, enables portal, and is accessible on open Internet
- Webcam sourcing video from pi cam, embedded in a single client browser window

### Self-Assessment 

| Objective Criterion | Rating | Max Value  | 
|---------------------------------------------|:-----------:|:---------:|
| Carmins connected via WiFi | 1  |  1     | 
| Data from each Carmin sent to central server and aggregated | 1  |  1     | 
| Portal reports live leader status and charts on web site | 1  |  1     | 
| Central server reports live leader status back to Carmin alpha displays | 1 |  1     | 
| Portal accessible from open internet | 1 |  1     | 
| Web cam operational in same browser window at client | 1 |  1     | 
| Node.js runs on pi | 1 |  1     | 

### Solution Design
In Quest 3, our goal was to aggregate data from multiple Carmins through wireless networks.

#### WiFi Connection
To ensure the Carmins were connected on WiFi, we created a free DDNS account from NO-IP provider and set up local "Group 7" router to connect to the service. Then we setup our laptop as a web server (node.js server) and accessed the web server from the hosted DDNS name (on and off the BU network, e.g., from your cellular service). We made our hostname: group7.ddns.net and brought up basic WiFi functionality on an ESP32 by altering the WIFI_SSID.

#### Web Portal
Data was plotting is done through CanvasJS which followed (1) Feeding the (live) data from the serial port directly into the browser client via socket.io as the link between the node app and the client javascript; and (2) feeding the live data to the browser while saving the data to a local file (local to the node server). The ESP32 writes sensor data to the serial port which is read by the JS file which then writes the sensor data into a csv file (for storage purposes), which is then read and updated to the nodeJS server. The nodeJS server checks for after updates to the csv file and pushes it to the sensor graphs in real-time. 

#### Time Display
For time tracking we used an alphanumeric display that interfaces with the ESP32 using I2C. The time in the present is sent to the serial port from the nodeJS server once and is kept track of and incremented using the ESP32s own time tracking functionality.

#### Temperature and Buzzer
We used a thermistor to measure temperature, converted ADC values from the thermistor into Celsius values, once the Celsius values exceeded a certain set threshold (40 C) the buzzer would go off.

#### Activity Tracking
Activity tracking was done by a single button cycling between states. Upon first press the sampling process would start and sensor data would be processed and displayed. Upon second press the sampling would stop. Upon third press the data was reset. The cycle would continue to loop **start->stop->reset->start->...**

#### Dynamic Plotting of Data
To plot our data we used similar strategy to Quest 2 with a nodeJS server along with the CanvasJS graphing tool to dynamically plot our data. The JS code uses ``fswatch()`` to watch for any changes to the csv file and as soon as and since the JS writes new sensor data to the csv line by line, when the csv file updates the JS will read the data and push it to the server using socket.io. The only changes it that this time we added a new fuunctionality of the leader, for this we made a bar graph from the CanvasJS library and pushed a variable that constantly kept track of the name and total step value of the leader.

### Sketches/Diagrams

![image](https://github.com/BU-EC444/Team7-Lee-Li-Slobodchikov-Sui/assets/114517092/4ff96b6c-e78b-48b5-8e79-bf62f00620cb)


### Supporting Artifacts

[![Straba Demo](<images/Screenshot 2023-10-26 at 9.09.18 PM.png>)](https://drive.google.com/file/d/1yyxFPOEBhLKcMl3_Dy2q6e8vETdw8cQn/view?usp=sharing)
<p align="center">
<i>Straba Demo</i>
</p>

[![Straba Technical](<images/Screenshot 2023-10-26 at 9.09.35 PM.png>)](https://drive.google.com/file/d/1aAKkc4bS0-rHjh72BFBYsLzd88dr5FB7/view?usp=sharing)
<p align="center">
<i>Straba Technical</i>
</p>


### Modules, Tools, Source Used Including Attribution
- [WiFi Station Example](https://github.com/espressif/esp-idf/tree/master/examples/wifi/getting_started/station)
- [UDP client](https://github.com/espressif/esp-idf/tree/master/examples/protocols/sockets/udp_client)
- [Serial Port Module](https://www.npmjs.com/package/serialport)
- [Socket.io Brief](/docs/briefs/design-patterns/dp-socketIO.md)
- [Serial to Node Example](https://github.com/BU-EC444/04-Code-Examples/tree/main/serial-esp-to-node-serialport)
