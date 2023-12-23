const express = require('express');
const http = require('http');
const socketIo = require('socket.io');
const {SerialPort} = require('serialport');
const fs = require('fs');
const Papa = require('papaparse');
const port = new SerialPort({ path: '/dev/cu.usbserial-0264FEBE', baudRate: 115200 });

const app = express();
const server = http.createServer(app);
const io = socketIo(server);

const {ReadlineParser} = require('@serialport/parser-readline');

const parser = port.pipe(new ReadlineParser({ delimiter: '\n' }));
const textFileStream = fs.createWriteStream('data.csv', { flags: 'a' });

const { DateTime } = require('luxon');
let now = DateTime.now();
let currentTime = now.toFormat('HH:mm:ss');

app.use(express.static(__dirname));  // Serve static files

function getLastDataFromCSV(filePath) {
    const csvContent = fs.readFileSync(filePath, 'utf-8');
    const parsed = Papa.parse(csvContent, { header: true });
    return parsed.data[parsed.data.length - 2];
}

const csvFilePath = 'data.csv';
fs.watch(csvFilePath, (eventType, filename) => {
    if (eventType === 'change') {
        const newEntry = getLastDataFromCSV(csvFilePath);
        io.emit('data', {
            x: parseFloat(newEntry.Time),
            y: parseFloat(newEntry.Step)
        });
    }
});

server.listen(3000, () => {
    console.log('listening on *:3000');
});

port.on("open", () => {
    textFileStream.write('Time,Step,Temp\n');
    });
    parser.on('data', data =>{

        let content;
        now = DateTime.now();
        currentTime = now.toFormat('HH:mm:ss');
        content = currentTime + "," + data + '\n';
        console.log(typeof data);

        if(data == "CLEAR")
        {
            console.log("attempt clear");
            //fs.writeFile('data.csv', '', function(){console.log('done')})
            fs.truncate('data.csv', 0, function(){console.log('done')})
            io.emit('refresh'); 

        }
        else
        {
            textFileStream.write(content);
        }
    });

const parsedTime = currentTime.split(":");
const dataToSend = parsedTime[0] + parsedTime[1]; // Replace with the data you want to send
port.write(dataToSend, (err) => {
    if (err) {
        console.error('Error writing to serial port:', err);
    } else {
        console.log(`sent: ${dataToSend}`);
    }
});

