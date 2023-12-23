# Smart Voting 

Authors: Jake Lee, Jason Li , JiaLin Sui, Maxim Slobodchikov

Date: 2023-12-12

### Summary
The Smart Voting is a system leverages MQTT5 technology to establish voting stations enabling users to cast their votes for their preferred Northeastern states. Options include Connecticut, Maine, New Hampshire, New York, Rhode Island, and Vermont.

<p align="center">
<img src="./images/IMG_7289.jpg" width="50%">
</p>
<p align="center">
Secure Voting System (Voter Booths and Fobs)
</p>

### Self-Assessment 

| Objective Criterion | Rating | Max Value  | 
|---------------------------------------------|:-----------:|:---------:|
| Using at least 3 ESP32s running MQTT protocol | 1 |  1     | 
| Using MQTT Message Broker on Pi | 1 |  1     | 
| Multiple Clients can subscribe to Message Broker | 1 |  1     | 
| Client displays tabluated values from each ESP32 | 1 |  1     | 
| Client displays live visualization of device values as graphic (map, chart, etc.) | 1 |  1     | 
| Client runs on external network | 1 |  1     | 
| Client is able to change status of LED on each ESP32 | 1 |  1     | 


### Solution Design

#### Key Fob
Code enables the keyfob to communicate using infrared signals and UART (Universal Asynchronous Receiver-Transmitter), and also allows it to connect to a network via WiFi.

- Infrared Communication: Allows the keyfob to send and receive data via infrared signals.
- UART Communication: Facilitates wired data transmission and reception.
- WiFi Connectivity: Enables the keyfob to connect to a network for additional functionalities.
- Traffic Light Indication: Uses RGB LEDs to display different states like a traffic light system.
- Device ID Display: Features a unique way to show the device's ID using the onboard LED.
- User Interaction: Includes a button to change the device's ID.

The keyfob, once programmed with this code, operates automatically. It can send and receive data, change its display based on the received information, and communicate over a network. A user can interact with it via a button to change its ID.

#### MQTT ESP

This code implements a voting system using an ESP32 microcontroller. It integrates MQTT for network communication, allowing the system to publish and subscribe to topics related to voting. The application is designed to register votes from users, display voting states, and handle restrictions on voting.

- Voting and State Selection Buttons: Utilizes GPIOs for user inputs to register votes and toggle through different states.
- MQTT Communication: Leverages MQTT protocol for sending and receiving voting data over a network.
- Alphanumeric Display: Displays the current state on a 14-segment display using I2C communication.
- Restricted Voting: Maintains a list of restricted IDs, preventing them from voting.
- LED Feedback: Provides visual feedback using RGB LEDs based on voting and system status.
- Infrared Communication: Integrates IR transmission and reception for additional functionalities.

##### Functionality

- Vote Registration: A button allows users to submit their vote, which is then published to an MQTT topic.
- State Toggle: Another button enables users to cycle through predefined states (like Connecticut, Massachusetts, etc.).
- MQTT Operations: Connects to an MQTT broker to publish votes and subscribe to topics that restrict voting based on user IDs.
- Display State: The current state is displayed on an alphanumeric screen.
- ID Reception: Receives and processes IDs via UART, checking against a list of restricted IDs.
- LED Indicators: Changes LED colors to indicate the system status and vote validity.

#### MQTT JS

This Node.js application implements a real-time voting system utilizing MQTT for messaging, Express for serving web content, and Socket.io for real-time client-server communication. It monitors and updates vote counts in a CSV file and communicates with MQTT clients.

- Real-time Vote Monitoring: Listens for changes in a CSV file and updates connected clients using Socket.io.
- MQTT Integration: Connects to an MQTT broker to receive voting data and send restrictions.
- Web Server with Express: Hosts a static website and serves an HTML file for displaying voting data.
- CSV File Handling: Reads and updates a CSV file containing voting data.

- Vote Receiving via MQTT: Receives votes sent to an MQTT topic and processes them.
- Vote Count Update: Parses and increments vote counts in a CSV file based on received MQTT messages.
- Client Notification: Notifies web clients in real-time of vote updates using Socket.io.
- Web Interface: Serves a web interface where users can view the voting results.
- File Watcher: Monitors changes in the CSV file to trigger client updates.

#### HTML Votemap

VoteMap is a web application designed for live visualization of polling data across different states. It uses D3.js for rendering an interactive map, Socket.io for real-time updates, and a backend that monitors vote counts in a CSV file. This application provides a dynamic, user-friendly interface to display and interact with live polling data.

- Interactive Map Visualization: Utilizes D3.js to render an interactive map of the United States, displaying voting data per state.
- Real-time Data Updates: Socket.io integration enables real-time updates to the map as new votes are recorded.
CSV File Monitoring: Back-end logic watches for changes in a CSV file containing vote counts, triggering updates to the front end.
- Responsive Design: Adapts to different screen sizes for optimal viewing on various devices.

- Map Rendering: Upon loading, the application renders a map using GeoJSON data and colors each state based on the number of votes.
- Data Processing: Reads voting data from a CSV file and maps it to the corresponding states on the map.
- Real-time Interaction: Listens for 'file-updated' events via Socket.io to refresh the map and leaderboard with the latest data.
- Leaderboard Display: Shows the top three states with the highest vote counts in a dynamically updated leaderboard.


### Sketches/Diagrams
<p align="center">
<img src="./images/IMG_7300.JPG" width="50%">
</p>
<p align="center">
Secure Voting System Diagram
</p>



### Supporting Artifacts
- [![Secure Voting Technical Video](<images/Screenshot 2023-12-13 at 3.13.58 AM.png>)](https://drive.google.com/file/d/1uxM1krqYHxD_7CdXxKZS0DRKpa70xXmt/preview)
<p align="center">
<i>Secure Voting Technical Video</i>
</p>

- [![Secure Voting Demo Video](<images/Screenshot 2023-12-13 at 3.11.57 AM.png>)](https://drive.google.com/file/d/1ghjzkZa16mnAUYdUfV_81MKFCMbAJU_l/preview)
<p align="center">
<i>Secure Voting Demo Video</i>
</p>


### Modules, Tools, Source Used Including Attribution

- [ESP MQTT5 Example Code Of Data Exchange With Node](https://github.com/BU-EC444/04-Code-Examples/tree/main/mqtt5-exchange)
- [ESP MQTT API Guide](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/protocols/mqtt.html)
- []()
- []()
- []()
