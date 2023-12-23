const Db = require('tingodb')().Db;
const readline = require('readline');
const fs = require('fs');
const db = new Db('./db', {});
const collection = db.collection("parking_db");
const newCollection = db.collection("record")

const IP = "192.168.1.25";
const port = 8080;

const dgram = require('dgram');
const server = dgram.createSocket('udp4'); // 'udp4' for IPv4

const init_collection = db.collection("parking_db");

let datatoInsert = [
    { meterID: 0, fobID: 999, meterStatus: false, timestamp: Date.now() },
    { meterID: 1, fobID: 999, meterStatus: false, timestamp: Date.now() }
]

// Delete the file
fs.unlink('./db/parking_db', (err) => {
  if (err) {
    console.error('Error deleting file:', err);
  } else {
    console.log('File successfully deleted');
  }
});

// Delete the file
fs.unlink('./db/record', (err) => {
    if (err) {
      console.error('Error deleting file:', err);
    } else {
      console.log('File successfully deleted');
    }
  });

function startDb() {
    init_collection.insert(datatoInsert, function (err, result) {
        if (err) {
            console.log('Insert error:', err);
        } else {
            console.log('Insert successful');
        }
    });
}

const init_newCollection = db.collection("record")

let datalogInsert = [
    { meterID: 0, fobID: 999, status: "init", timestamp: Date.now(), ipAddress: "test.test.test" }
]

function startlogDb() {
    init_newCollection.insert(datalogInsert, function (err, result) {
        if (err) {
            console.log('Insert error:', err);
        } else {
            console.log('Insert successful');
        }
    });
}

startDb();
startlogDb();

const uniqueIdentifiers = new Set();
let listenForNewIPs = true;
const identifierToIPMap = new Map();

function lookupIP(idType, idValue) {
	const compositeKey = `${idType}:${idValue}`;
	if (identifierToIPMap.has(compositeKey)) {
		return identifierToIPMap.get(compositeKey);
	} else {
		return null; // or any appropriate response indicating no data found
	}
}

function logQueryRequest(meterID, fobID, status, ipAddress) {
	const queryLog = {
		meterID: meterID,
		fobID: fobID,
		status: status, // You can include more details like 'occupied', 'available', etc.
		timestamp: new Date(), // Current timestamp
		ipAddress: ipAddress
	};
	newCollection.insert(queryLog, function (err, result) {
		if (err) {
			console.log('Error logging query request:', err);
		} else {
			console.log('Query request logged:', result);
		}
	});
}


const rl = readline.createInterface({
	input: process.stdin,
	output: process.stdout
});

server.on('error', (err) => {
	console.log(`Server error:\n${err.stack}`);
	server.close();
});

server.on('message', (msg, rinfo) => {
	console.log(`Server received: ${msg} from ${rinfo.address}:${rinfo.port}`);

	const data = msg.toString();

	if (listenForNewIPs) {
		let [idType, idValue] = data.split(",").map(part => part.trim());
		idValue = parseInt(idValue);

		// Construct a unique combination string
		const uniqueCombination = `${idType}:${idValue}:${rinfo.address}`;

		// Check if the combination is unique
		if (!uniqueIdentifiers.has(uniqueCombination)) {
			uniqueIdentifiers.add(uniqueCombination); // Add the new combination to the Set
			server.send("1", 4000, rinfo.address); // Send "1" as response
		}

		const compositeKey = `${idType}:${idValue}`;
		identifierToIPMap.set(compositeKey, rinfo.address);
	}
	///const data = msg.toString();
	let [newMeterID, newFobID] = data.split(",").map(part => part.trim());
	newMeterID = parseInt(newMeterID);
	newFobID = parseInt(newFobID);

	collection.find({ meterID: newMeterID }).toArray(function (err, items) {
		if (err) {
			console.log('Error querying the database:', err);
			return;
		}

		if (items.length > 0) {
			console.log(`Found ${items.length} items with meterID ${newMeterID}`);
			items.forEach(item => {
				console.log(`fobID for item with meterID ${newMeterID}: ${item.fobID}`);
			});

			// Assuming the first item is the one we're interested in
			let currentItem = items[0];

			if (currentItem.meterStatus == 0) {
				collection.update({ meterID: newMeterID }, { $set: { fobID: newFobID, meterStatus: 1 } }, { upsert: false }, function (updateErr, updateResult) {


					logQueryRequest(newMeterID, newFobID, 'occupied', rinfo.address);

					if (updateErr) {
						console.log('Error updating the record:', updateErr);
						server.send("Error updating record", 4000, rinfo.address); // Sending error message back to client
						server.send("Error: No Status Changes", 4000, lookupIP("FobID",newMeterID))
						server.send("Error: No Status Changes", 4000, lookupIP("MeterID", newMeterID))
					} else {
						console.log('Meter status updated to occupied:', updateResult);
						server.send("Meter status updated to occupied", 4000, rinfo.address); // Sending success message back to client
						server.send("TAKEN", 4000, lookupIP("FobID",newMeterID))
						server.send("TAKEN", 4000, lookupIP("MeterID", newMeterID))
					}
				});
			} else if (currentItem.meterStatus == 1) {

				logQueryRequest(newMeterID, newFobID, 'available', rinfo.address);

				if (currentItem.fobID == newFobID) {
					collection.update({ meterID: newMeterID }, { $set: { fobID: newFobID, meterStatus: 0 } }, { upsert: false }, function (updateErr, updateResult) {

						if (updateErr) {
							console.log('Error updating the record:', updateErr);
							server.send("Error updating record", rinfo.port, rinfo.address); // Sending error message back to client
							server.send("Error: No Status Changes", 4000, lookupIP("FobID",newMeterID))
							server.send("Error: No Status Changes", 4000, lookupIP("MeterID", newMeterID))

						} else {
							console.log('Meter status updated to available:', updateResult);
							server.send("Meter status updated to available", rinfo.port, rinfo.address);
							server.send("OPEN", 4000, lookupIP("FobID",newMeterID))
							server.send("OPEN", 4000, lookupIP("MeterID", newMeterID))
						}
					});
				} else {
					console.log('Spot has been taken');
					server.send("Spot has been taken", rinfo.port, rinfo.address); // Sending spot taken message back to client
					server.send("TAKEN", 4000, lookupIP("FobID",newMeterID))
					server.send("TAKEN", 4000, lookupIP("MeterID", newMeterID))
				}
			}
		} else {
			console.log('No records found with meterID:', newMeterID);
			server.send("No records found with the specified meterID", rinfo.port, rinfo.address); // Sending no record message back to client
		}
	});
});

server.on('listening', () => {
	const address = server.address();
	console.log(`Server listening on ${address.address}:${address.port}`);
});

server.bind(port, IP);

rl.on('line', (input) => {
	console.log(`Received: ${input}`);
	if (input === 'x') { // Change this condition to choose a different keystroke if needed
		listenForNewIPs = false; // Stop listening for new IPs
		console.log('Server will no longer listen for new IP addresses.');
	}
});

const express = require('express');
const app = express();
const path = require('path');

app.use(express.static(path.join(__dirname, 'public')));

// Define a route for the homepage
app.get('/', (req, res) => {
	res.sendFile(path.join(__dirname, 'public', 'index.html'));
});

app.get('/parking-status', (req, res) => {
collection.find({}).toArray((err, items) => {
	if (err) {
	console.log('Error querying the database:', err);
	res.status(500).send('Error querying the database.');
	return;
	}

	if (items.length > 0) {
	// console.log(`Found ${items.length} parking spots`);

	// Array to store the processed parking status
	let parkingStatus = [];

	// Process each parking spot
	items.forEach(item => {
		// console.log(`fobID for parking spot with meterID ${item.meterID}: ${item.meterStatus}`);

		// Assuming each item is a parking spot of interest
		let currentItem = item;

		// Add the meterID and meterStatus of the current item to the parkingStatus array
		parkingStatus.push({
		meterID: currentItem.meterID,
		meterStatus: currentItem.meterStatus // This will be 0 or 1
		});
	});

	// Send the parkingStatus array as a JSON response
	res.json(parkingStatus);
	} else {
	console.log('No parking spots found');
	res.status(404).send('No parking spots found');
	}
});
})
  
// Start the server
const web_port = process.env.PORT || 6000;
app.listen(web_port, () => {
   console.log(`Website running on http://localhost:${web_port}`);
});