# Straba Quest Code Readme


### NodeJS and Server, Step and Temperature Visualization using Express, Socket.io, and CanvasJS

Our web app visualizes real-time step and temperature data from CSV files.

Technologies:
1. Node.js & Express: Backend server setup.
2. Socket.io: Real-time server-client communication.
3. CanvasJS: Client-side data visualization.
4. PapaParse: CSV parsing

Server-side (server.js):

Watches for changes in the specified CSV files using the fs.watch method.
When a change is detected, it reads and parses the CSV using PapaParse to extract the relevant data.
Uses Socket.io to emit the new data to any connected clients.
Keeps track of which carmin csv file has the largest total steps and updates to the graph accordingly. The server also listens for any esp's that are trying to send sensor data. Once data is received, it appends the data to the corresponding esp's ip address csv file name. It then calculates which esp is leading the top amount of steps and returns the message along with time of day to the esps.

Client-side (index.html):

Listens for data updates from the server using Socket.io.
Updates CanvasJS charts in real-time when new data is received.
Displays a comparison bar chart for total steps of each ESP watch
Displays the leader and the total steps and the ID of the leader.

Esp32:

The esp code is built on-top of our last skill. Each of our esp's/carmin watches are also wifi-supported now, so we configured them to connect to our router and send sensor data across UDP sockets. It then receives information from the server regarding the current leader and time of day. This is displayed on the alphanumeric display, and since the total character length of the entire message is more than 4, we had to scroll the message.
