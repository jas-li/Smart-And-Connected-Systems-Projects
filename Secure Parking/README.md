# Secure Parking

Authors: Jake Lee, Jason Li , JiaLin Sui, Maxim Slobodchikov

Date: 2023-12-06

### Summary


For this quest we implemented a parking meter system, comprised of ESP32-powered parking meters and key fobs. It integrates infrared communication, QR code functionality, and LED indicators for parking space allocation. The system communicates with an authentication server for efficient space management, which is handled through a Node.js script. Key features include Wi-Fi connectivity, peripheral integration, and task management using FreeRTOS, making the system an innovative solution for parking management.

### Self-Assessment 

| Objective Criterion | Rating | Max Value  | 
|---------------------------------------------|:-----------:|:---------:|
| Fob sends (IR TX/RX) FID to meter | 1 |  1     | 
| Meter ESP displays QR code with MID,FID | 1 |  1     | 
| Fob Pi reads QR code on meter and sends (WiFi) to Auth Server with MID,FID | 1 |  1     | 
| DB logs MID,FID,timestamp and validates meter available; sends (WiFi) MID, status to fob and meter | 1 |  1     | 
| Meter evaluates status and sets LEDs to indicate booking | 1 |  1     | 
| Web client accesses Auth Server to show logged data and meter status | 1 |  1     | 
| Auth Server implemented in TingoDB and on separate Pi | 1 |  1     | 
| nvestigative question response | 1 |  1     | 


### Solution Design

#### Parking Meter
The parking meter has a OLED and a IR receiver that waits for a signal. Once a signal with the FobID is received the OLED is turned on the LED along with changing the LED to blue to indicate a loading state. The OLED display a generated bitmap QR code that contains the MeterID along with the FobID.  

#### Key Fob
The key fob is an IR transmitter that sends a signal. On button press the LED indicator is changed to LOADING (BLUE) and the IR signal is sent with the FobID. 

#### Database
The data is initialized using `query.js` which creates two databases one for logging the every time the database gets queryed for authentication and one for keep tracking of the unique IP addresses of the meters and fobs.

#### Authentication and Relaying Commands
`query.js` is also the code that constantly listens for requests and is the part of the system that has constant uptime. Upon initialization of the fobs and meters it pings the authentication server, if the ping is successful it sends a message that identifies the fob or meter with the device type (fob or meter) and the ID of the fob or meter. Once the server as received this message it logs the IP address it came from along with its associated IP address. Only unique IP addresses are stored in memory. Upon receiving an OK from the server recognising the device that the device is registered, the devices' ESP32 no become constant listeners awaiting instructions.

Once a query request is send through via the scanning of a QR code it sends a request to the server which broadcasts instructions only to the relevant fob and meter to change their status LEDs. For both meter and fob it sends a message **TAKEN** or **OPEN**. For the fob **TAKEN** means that the fob is associated with a meter and the green LED turns on. For the meter **TAKEN** means that the meter is taken and the RED of the meter turns on and vice versa. **OPEN** -> green LED for meter. **OPEN** -> red LED for fob.


### Sketches/Diagrams
<p align="center">
<img src="./images/IMG_7259.jpg" width="50%">
</p>
<p align="center">
Overall Diagram
</p>



### Supporting Artifacts
- [Link to video technical presentation](https://drive.google.com/file/d/1Av_ksxXb-KBFu_J8EJVZjGKLurygRgaE/view?usp=sharing).

[![Smart Meter Demo Video](<images/Screenshot 2023-12-09 at 6.31.35 PM.png>)](https://drive.google.com/file/d/1yjosqOvlYP88z4tF5V-4GahG3O_atsyN/view?usp=sharing)
<p align="center">
<i>Secure Parking Demo Video</i>
</p>



### Modules, Tools, Source Used Including Attribution
- [IR Communication Design Pattern](https://github.com/BU-EC444/01-EBook/blob/main/docs/design-patterns/docs/dp-irtxrx.md)
- [Tingo DB Recipe For Sensor Data](https://github.com/BU-EC444/01-EBook/blob/main/docs/recipes/docs/tingo.md)
- [Sample Code For OLED Bitmap Graphics Display](https://github.com/BU-EC444/04-Code-Examples/tree/main/oled-qr-display)



