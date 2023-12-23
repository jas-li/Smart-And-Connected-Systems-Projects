const mqtt = require('mqtt')

const fs = require('fs');
const csv = require('csv-parser');
const { format } = require('fast-csv');

const express = require('express');

const app = express();
const http = require('http').createServer(app);
const web_port = 3000; // You can choose any port number

const io = require('socket.io')(http);

function delay() {
  return (resolve => {
      setTimeout(() => {
          resolve();
      }, 2000);
  });
}

fs.watch('./public/votes.csv', (eventType, filename) => {
  if (filename && eventType === 'change') {
    io.emit('file-updated'); // Notify all connected clients
    delay();
  }
});

app.use(express.static('public'));

// Alternatively, serve the HTML file directly for a specific route
app.get('/', (req, res) => {
  res.sendFile(__dirname + '/public/index.html');
});

// Start the server
// app.listen(web_port, () => {
//   console.log(`Server running at http://localhost:${web_port}`);
// });
http.listen(3000, () => {
  console.log('Listening on *:3000');
});

const inputFile = './public/votes.csv';

const protocol = 'mqtt'
// const host = '192.168.1.36' // your broker (pi) ip address
const host = 'group7.ddns.net' // your broker (pi) ip address
const port = '1883'
const clientId = `mqtt_${Math.random().toString(16).slice(3)}`

const connectUrl = `${protocol}://${host}:${port}`

const client = mqtt.connect(connectUrl, {
    clientId,
    protocolVersion: 5, // using MQTT v5
})

const subscribe_topic = '/topic/votes'
const publish_topic = '/topic/restrict'

client.on('connect', () => {
  console.log('Connected')
  // Subscribe to a topic
  client.subscribe([subscribe_topic], () => {
    console.log(`Subscribe to topic '${subscribe_topic}'`)
  })
})

client.on('message', (subscribe_topic, payload) => {
    console.log('Received Message:', subscribe_topic, payload.toString())
    let recv_msg = payload.toString().split(",");

    // update vote to csv using recv_msg[0]
    const stateToUpdate = recv_msg[0]; // Replace with the state you want to update
    const incrementValue = 1; // The value by which you want to increment the votes
    let data = [];

    // Read the CSV file and process each row
    fs.createReadStream(inputFile)
    .pipe(csv())
    .on('data', (row) => {
      if (row.state === stateToUpdate) {
        // Increment the votes for the specified state
        row.votes = parseInt(row.votes, 10) + incrementValue;
      }
      data.push(row);
    })
    .on('end', () => {
      console.log('CSV file successfully processed');

      // Overwrite the original file with updated data
      const ws = fs.createWriteStream(inputFile);
      ws.on('finish', () => console.log('CSV file has been updated'));

      const csvStream = format({ headers: true });
      csvStream.pipe(ws);

      data.forEach(row => csvStream.write(row));
      csvStream.end();
    });
    setTimeout(() => {
        // Publish a message to a topic
        client.publish(publish_topic, recv_msg[1])
        console.log('Published Message:', publish_topic, recv_msg[1])
    }, 200)
})