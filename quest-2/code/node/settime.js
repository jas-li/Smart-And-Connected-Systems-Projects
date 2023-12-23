const {SerialPort} = require('serialport');
const { DateTime } = require('luxon');

const port = new SerialPort({ path: '/dev/cu.usbserial-0264FEBE', baudRate: 115200 });

// add
const express = require('express');
const http = require('http');
const socketIo = require('socket.io');
const fs = require('fs');
const Papa = require('papaparse');
const app = express();
const server = http.createServer(app);
const io = socketIo(server);

const {ReadlineParser} = require('@serialport/parser-readline');

const parser = port.pipe(new ReadlineParser({ delimiter: '\n' }));
const textFileStream = fs.createWriteStream('data.csv', { flags: 'a' });
//

function sendTime() 
{
    let now = DateTime.now();
    let currentTime = now.toFormat('HH:mm:ss');
    
    const parsedTime = currentTime.split(":");
    const dataToSend = parsedTime[0] + parsedTime[1];
    
    port.write(dataToSend, (err) => {
        if (err) {
            console.error('Error writing to serial port:', err);
        } else {
            console.log(`sent: ${dataToSend}`);
    
            // Add a delay before reading from the serial port
            setTimeout(() => {
                readFromPort();
            }, 1000);  // For example, delay of 1 second
        }
    });
    
}

setInterval(sendTime, 5000);

port.on("open", () => {
    textFileStream.write('Time,Step,Temp\n');
    });
    parser.on('data', data =>{
        let content;
        now = DateTime.now();
        currentTime = now.toFormat('HH:mm:ss');
        content = currentTime + "," + data + '\n';
        console.log(data);
        //console.log(data.localeCompare("CLEAR"));
        if(data.localeCompare("CLEAR") == 1)
        {
            console.log("attempt clear");
            fs.writeFile('data.csv', "Time,Step,Temp\n", function(){console.log('done')})
            // fs.truncate('data.csv', 0, function(){console.log('done')});
            console.log("attempt rewrite");
            textFileStream.write('Time,Step,Temp\n');
            io.emit('refresh'); 
        }
        else
        {
            textFileStream.write(content);
        }
    });

    function readFromPort() {
    port.once('data', (data) => {
        console.log(`Received: ${data.toString()}`);
    });
}

// canvas
app.use(express.static(__dirname));  // Serve static files

function getLastDataFromCSV(filePath) {
    const csvContent = fs.readFileSync(filePath, 'utf-8');
    const parsed = Papa.parse(csvContent, { header: true });
    return parsed.data[parsed.data.length - 2];
}

const csvFilePath = './data.csv';

function timeStringToDate(timeStr) {
    const [hours, minutes, seconds] = timeStr.split(":").map(Number);
    const date = new Date();
    date.setHours(hours, minutes, seconds, 0);
    return date;
}

fs.watch(csvFilePath, (eventType, filename) => {
    if (eventType === 'change') {
        
        const newEntry = getLastDataFromCSV(csvFilePath);
        
        // Parse the string to get a Date object
        if(newEntry == undefined)
        {
            return;
        }
        
        const parsedTime = timeStringToDate(newEntry.Time);
        
        io.emit('data', {
            x: parsedTime,
            y: parseFloat(newEntry.Step),
            temp: parseFloat(newEntry.Temp)
        });
    }
});

server.listen(3000, () => {
    console.log('listening on *:3000');
});
