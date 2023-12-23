// Modules
var dgram = require('dgram'); // Socket programming
const { DateTime } = require('luxon'); // Get the current time of day
var fs = require('fs'); // To save to a file
let now = DateTime.now();

// Port and IP
var PORT = 3333; // Initialize a port
var HOST = '192.168.1.36'; // Ip address of pi

// Create a CSV file
fs.writeFile('data.csv', '', function (err) {
    if (err) throw err;
});

// Create socket
var server = dgram.createSocket('udp4');

// Create server that listens on a port
server.on('listening', function () {
    var address = server.address();
    console.log('UDP Server listening on ' + address.address + ":" + address.port);
});

// ------ Initialize some variables to track leaderboard stats ------
let carminID = []; // Track which carmins are connected
let count = 0;
let stepsArr = [];
let tempArr = [];

// initialize leaderboard
let leaderboard = [];


// On connection, print out received message
server.on('message', function (message, remote) {
  let carminData = remote.address + ':' + remote.port + "-" + message; // Later parse message by "," to get the sensor contents
  console.log(carminData);

  // Save carmin watch data to CSV in format IPaddress:Port-Sensor,Sensor
  fs.appendFile('data.csv', carminData, function (err) {
      if (err) throw err;
  });

  // ------ Add some leaderboard/parsing logic ------
  // Parser
  let data = message.toString();
  data = data.split(",");
  stepsArr.push(parseInt(data[0])); // Push the new steps recieved into array of steps
  tempArr.push(parseFloat(data[1])); // Push new temps into array

  // Update leaderboard data
  leaderboard.push({
      ipAddress: remote.address,
      port: remote.port,
      steps: parseInt(data[0]),
      temperature: parseFloat(data[1]),
  });

  // Sort the leaderboard based on steps (descending order)
  leaderboard.sort((a, b) => b.steps - a.steps);

  // Prepare the leaderboard message (by steps)
  let leaderboardMessage = "Leaderboard:\n";
  leaderboard.forEach((entry, index) => {
      leaderboardMessage += `${index + 1}. ${entry.ipAddress}:${entry.port} - Steps: ${entry.steps}, Temp: ${entry.temperature}\n`;
  });

  // Send leaderboard information
  server.send(leaderboardMessage, remote.port, remote.address, function (error) {
      if (error) {
          console.log('MEH!');
      } else {
          console.log('Sent Leaderboard:\n', leaderboardMessage);
      }
  });

  // Limit the leaderboard to the top 10 entries to prevent overflow
  if (leaderboard.length > 10) {
      leaderboard.pop(); // Remove the last entry
  }
});

// Bind server to port and IP
server.bind(PORT, HOST);

