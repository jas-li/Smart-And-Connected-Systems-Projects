# Quest 5: Code Readme

## ESP code (Parking Meter)
blink_example_main.c contains logic for the ESP32 board system designed to manage a parking space allocation system using a combination of IR communication, QR code scanning, and Wi-Fi networking. It integrates various hardware components and communication protocols to achieve a sophisticated parking management solution. The main components and functionalities of this code include:

1) Infrared Transmission and Reception: It uses IR transmission and reception to exchange data between a fob and a meter. The fob sends an ID to the meter, which then generates a QR code containing both meter and fob IDs.

2) QR Code Generation and Scanning: The meter generates a QR code, and the fob scans this QR code to obtain the meter and fob IDs.

3) Communication with Authentication Server: The fob sends these IDs to an Authentication server, which validates them and returns a meter key and status.

4) Parking Space Allocation: Depending on the meter's status, parking is allocated, indicated by LED statuses. The system also supports web client access for monitoring the status of each parking space.

5) Peripheral Integration: The code includes setups for various peripherals like PWM (Pulse Width Modulation) for IR transmission, UART (Universal Asynchronous Receiver/Transmitter) for serial communication, LEDs for status indication, and an OLED display for showing QR codes or other information.

6) Wi-Fi Connectivity: It uses Wi-Fi for network communication, presumably to connect with the authentication server and for web client interactions.

7) Task Management: The code employs FreeRTOS for task management, creating separate tasks for handling different functionalities like IR signal reception, LED control, QR code display, and network communication.

8) GPIO Configuration: General Purpose Input/Output (GPIO) pins are configured for various functions like button inputs, LED outputs, and communication interfaces.

9) UDP Server and Client: Implements a UDP server and client for network communication, possibly to receive commands or send data to a central server.

## ESP code (Key Fob)
This code handles IR and UART communication in an ESP32 environment, demonstrating integration with peripherals, Wi-Fi, and RTOS task management.

1) Peripheral Initialization and Configuration:
- Sets up PWM (Pulse Width Modulation) for IR transmission at a frequency of 38kHz.
- Initializes UART (Universal Asynchronous Receiver/Transmitter) for serial communication, with designated pins for transmitting and receiving.
- Configures GPIO (General-Purpose Input/Output) pins for different components like LEDs, a button, and hardware interrupts.
  
2) LED Control:
- Manages RGB LEDs to display different colors based on the system state or external inputs.
- Utilizes an onboard LED to indicate device ID through blinking patterns.

3) Button Interaction:
- Implements a button to toggle the sending state or change device ID.
- Uses hardware interrupts to handle button presses.

4) IR and UART Communication:
- Sends and receives data packets via UART, including a start byte, device ID, color information, and a checksum for data integrity.
- Note: The code suggests it's not reliable to run both IR transmitter (TX) and receiver (RX) simultaneously.

5) Wi-Fi Setup and UDP Networking:
- Initializes Wi-Fi connectivity with specified SSID and password.
- Creates UDP server and client tasks for network communication, handling incoming and outgoing UDP packets.

6) RTOS Task Management:
- Implements several tasks for different functionalities like sending data, receiving data, managing LEDs, and handling button input.
- Includes a timer task for time-based actions (commented out in this code).

7) Main Function (app_main):
- Initializes the NVS (Non-Volatile Storage), Netif (Network Interface), and Event Loop for ESP32.
- Calls initialization functions for various peripherals and creates RTOS tasks.

## auth_server.js

The Node.js script primarily functions as a server for managing parking space allocation and tracking using a combination of database operations, UDP networking, and web server functionalities.

1) Database Setup and Management:

- Utilizes tingodb, a lightweight, file-based database, for data storage.
- Initializes two collections within the database: parking_db and record.
- Inserts initial data into these collections.
- Deletes existing database files for parking_db and record at startup.

2) UDP Server Configuration:

- Creates a UDP server for network communication, listening for messages on a specific IP and port.
- Processes incoming messages to manage parking space allocation and track the status of each parking meter.

3) Parking Space Allocation Logic:
- Receives meter IDs and fob IDs via UDP messages.
- Checks and updates the status of parking meters (occupied or available) based on the received IDs.
- Logs each query request for future reference.

4) IP Address Mapping and Management:
- Maps unique identifiers (composed of meter ID and fob ID) to IP addresses.
- Includes functionality to stop listening for new IP addresses based on user input.

5) Web Server with Express:
- Sets up a web server using Express.js.
- Serves a static webpage and provides an endpoint (/parking-status) to retrieve the status of all parking spots in JSON format.

6) Readline Interface:
- Implements a command-line interface for additional user input and control.

7) Error Handling and Logging:
- Includes error handling for various operations, especially database interactions and UDP message processing.
- Logs significant events and errors to the console.
