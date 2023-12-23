# Venus Rover Code Documentation

Authors: Jake Lee, Jason Li , JiaLin Sui, Maxim Slobodchikov

Date: 2023-11-10

#### Code Overview:
Upon startup, the ESP32 initializes its Wi-Fi connection and starts the UDP server to listen for commands.
The LiDAR sensor continuously measures the distance in front of the buggy. If an obstacle is detected within a predefined range, the buggy stops or takes necessary action.
The ultrasonic sensors on both sides assist in navigating by constantly measuring the side distances. The steer_task uses these measurements to adjust the buggy's direction.
Speed is regulated based on encoder readings, and the servo motor adjusts the steering angle as per the PID controller's output.
Commands received via the UDP server (start, stop, turn) control the buggy's movements, allowing remote operation.
The alphanumeric display can show relevant information like speed, distance, or elapsed time.

#### Execution Flow:
Initialization: Wi-Fi and peripherals (like I2C, LiDAR, ultrasonic sensors) are initialized.
Task Creation: Multiple FreeRTOS tasks are created for handling different components (LiDAR, ultrasonic sensors, steering, speed control, display).
Operation: The buggy operates based on sensor inputs and received UDP commands. The LiDAR and ultrasonic sensors play a key role in avoiding obstacles, while the servo and motor are controlled to navigate and maintain speed.

#### UDP Commands and Responses:
When the UDP server receives a stop command, it activates an emergency brake (e_brake). The start command deactivates the emergency brake. The turn command initiates a 360-degree turn maneuver. This ESP32-based system showcases a sophisticated approach to robotic control, integrating various sensors and communication protocols for efficient and responsive operation.

#### Wireless Interface
<img width="500" alt="Screenshot 2023-11-10 at 11 40 41 PM" src="https://github.com/BU-EC444/Team7-Lee-Li-Slobodchikov-Sui/assets/93232189/5f12cc2a-654e-4c67-9bd8-08ca24c473d1">

1) Startup Instructions: To start server, we can cd into the Website folder and run "node server.js" in console. The site interface can then seen on http://localhost:3000/. One must remember to npm install express and node-fetch and ESP32 device needs to have a UDP server listening on port 8080 to receive messages "start", "stop", "turn".

2) Destination IP Address (ESP32_SERVER_IP): The messages are sent to the IP address specified by ESP32_SERVER_IP, which in your case is set to 'group7.ddns.net'. This is the dynamic DNS (DDNS) address that should resolve to the IP address of your network where the ESP32 device is located.

3) Destination UDP Port (ESP32_UDP_PORT): The messages are sent to the port number specified by ESP32_UDP_PORT, which is 8080 in your code. This means that your ESP32 device needs to have a UDP server listening on port 8080 to receive these messages.

4) Handling in the ESP32: The ESP32 should be programmed to run a UDP server that listens on the specified port (8080). When it receives a message on this port, it should interpret and act upon the message accordingly.

5) Process Flow: When a user interacts with the buttons on the web interface served from the public directory, an HTTP POST request is sent to the /control endpoint on your Node.js server.
The server then extracts the command from the request body (req.body.command). This command is converted to a string (String(req.body.command)) and sent as a UDP message to the ESP32 device at the specified DDNS address and port.



