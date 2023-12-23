// to start server cd into Website folder and run "node server.js" in console
// site interface can be seen on http://localhost:3000/
// remember to npm install express and node-fetch
// ESP32 device needs to have a UDP server listening on port 8080 to receive messages "start", "stop", "turn"

/*
README for quest:

Destination IP Address (ESP32_SERVER_IP): 
The messages are sent to the IP address specified by ESP32_SERVER_IP, which in your case is set to 'group7.ddns.net'. This is the dynamic DNS (DDNS) address that should resolve to the IP address of your network where the ESP32 device is located.

Destination UDP Port (ESP32_UDP_PORT): 
The messages are sent to the port number specified by ESP32_UDP_PORT, which is 8080 in your code. This means that your ESP32 device needs to have a UDP server listening on port 8080 to receive these messages.

Handling in the ESP32: 
The ESP32 should be programmed to run a UDP server that listens on the specified port (8080). When it receives a message on this port, it should interpret and act upon the message accordingly.

Process Flow:
When a user interacts with the buttons on the web interface served from the public directory, an HTTP POST request is sent to the /control endpoint on your Node.js server.
The server then extracts the command from the request body (req.body.command).
This command is converted to a string (String(req.body.command)) and sent as a UDP message to the ESP32 device at the specified DDNS address and port.

Network Configuration:
For the ESP32 to receive these messages, it must be connected to a network that is accessible via the group7.ddns.net address.
If the ESP32 is behind a router (which is usually the case), you'll need to set up port forwarding on your router to forward UDP traffic on port 8080 to the internal IP address of the ESP32 device.

Security Considerations:
Since your server sends commands to an ESP32 device over the internet, it's crucial to consider security implications, especially if the commands control a physical device.
Make sure to secure the communication and possibly authenticate the requests to prevent unauthorized access or control.
*/

const dgram = require('dgram');
const express = require('express');
const app = express();
const client = dgram.createSocket('udp4');

const ESP32_SERVER_IP = '192.168.1.23'; // Using DDNS
const ESP32_UDP_PORT = 3333;

app.use(express.json()); // Middleware to parse JSON
app.use(express.static('public'));


client.on('error', (err) => {
    console.error('Socket error:', err);
    client.close();
});

app.post('/control', (req, res) => {
    const message = String(req.body.command);
    console.log(`Preparing to send message: ${message}`);

    client.send(message, 0, message.length, ESP32_UDP_PORT, ESP32_SERVER_IP, (err) => {
        if (err) {
            console.error("Error sending message:", err);
            res.status(500).send('Error sending message to ESP32');
        } else {
            console.log("Message sent:", message);
            res.send('Message sent to ESP32');
        }
    });
});

client.on('listening', () => {
  const address = client.address();
  console.log(`UDP Client listening on ${address.address}:${address.port}`);
});

app.listen(3000, () => {
  console.log('HTTP server is listening on port 3000');
});

client.bind();  // Bind the client to start listening for input
