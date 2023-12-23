// Modules
var dgram = require('dgram'); // Socket programming
const { DateTime } = require('luxon'); // Get the current time of day
var fs = require('fs'); // To save to a file
let now = DateTime.now();

// Port and IP
var PORT = 3333; // Initialize a port
var HOST = '192.168.1.36'; // Ip address of pi
// var HOST = '10.239.114.40'; // Ip address of pi

// Clear all CSV files
const directoryPath = './';
fs.readdir(directoryPath, (err, files) => {
    if (err) {
        throw err;
    }

    // Filter the files with the .csv extension
    const csvFiles = files.filter(file => file.endsWith('.csv'));

    // Delete each of the filtered .csv files
    csvFiles.forEach(file => {
        fs.unlink(`${directoryPath}${file}`, err => {
            if (err) {
                throw err;
            }
        });
    });
});

// Create socket
var server = dgram.createSocket('udp4');

// Create server that listens on a port
server.on('listening', function () {
    var address = server.address();
    console.log('UDP Server listening on ' + address.address + ":" + address.port);
});

// ------ Initialize some variables to track leaderboard stats ------
let largestStepsFile = null;  // This will keep track of the file with the largest number of steps
let totalStepsMap = {};

let stepsArr = [];
let tempArr = [];

// Function to track most steps
function updateLargestStepsFile(filePath, totalSteps) {
    // Check if the key (filePath) exists in the map, and if not, initialize it with a value of 0
    if (!totalStepsMap[filePath]) {
        totalStepsMap[filePath] = 0;
    }

    totalStepsMap[filePath] += totalSteps;

    if (largestStepsFile === null || totalStepsMap[filePath] > totalStepsMap[largestStepsFile]) {
        largestStepsFile = filePath;
    }
}

// On connection, print out received message
server.on('message', function (message, remote) {
  let carminData = remote.address + ':' + remote.port + "-" + message; // Later parse message by "," to get the sensor contents
  console.log(carminData);

  // ------ Add some leaderboard/parsing logic ------
  // Parser
  let data = message.toString();
  data = data.split(",");
  stepsArr.push(parseInt(data[0])); // Push the new steps recieved into array of steps
  tempArr.push(parseFloat(data[1])); // Push new temps into array

  // Get time
  now = DateTime.now();
  let currentTime = now.toFormat('HH:mm:ss');
  let currentTimeParsed = currentTime.toString().split(':');
  let totalTime = currentTimeParsed[0] + currentTimeParsed[1];

  if (!fs.existsSync('PORT' + remote.port + '.csv')) {
    // If the file doesn't exist, create it and write the header row
    fs.writeFileSync('PORT' + remote.port + '.csv', 'Time,Step,Temp\n', function() {
        console.log('Created a file!');
    });
  }

  // Save carmin watch data to CSV in format IPaddress:Port-Sensor,Sensor
  if(message.toString().length != 0) {
      fs.appendFile('PORT' + remote.port + '.csv', currentTime + ',' + message, function (err) {
        if (err) throw err;
      });
  }

  updateLargestStepsFile('PORT' + remote.port + '.csv', parseInt(data[0]));
  let parseLargestPort = largestStepsFile.toString().split(".");
  parseLargestPort = parseLargestPort[0];

  // Send leaderboard information
  server.send("TIME-" + totalTime + ' ' + parseLargestPort, remote.port, remote.address, function (error) {
      if (error) {
          console.log('MEH!');
      } else {
          console.log('Sent: ', totalTime + ' ' + parseLargestPort);
      }
  });
});

// Bind server to port and IP
server.bind(PORT, HOST);