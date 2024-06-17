const http = require('http');
const url = require('url');
const fs = require('fs');
const path = require('path');
const util = require('util');
const { exec } = require('child_process');
const pexec = util.promisify(exec);

const port = 10000;
const pathEndpoint = '/KlingShell';
const enableShellCommands = false;

let consoleData = {};
let commandQueue = {}; // Store commands for devices
let deviceLastContact = {}; // Track last contact time for each device

function processMessageColors(message) {
    return message
        .replace(/\[#([0-9A-Fa-f]{6})\](.*?)((?=\[#)|$)/g, (match, p1, p2) => {
            return `<span style="color:#${p1}">${p2}</span>`;
        })
        .replace(/\[#\]/g, '</span>'); // Reset color
}

async function handlePayload(payload, handler, deviceId) {
    let response = {};

    if (handler === 'clear') {
        response.message = 'Manual clear not allowed from device';
    } else if (handler === 'shell' && enableShellCommands) {
        const command = payload;
        console.log(`Remote shell: '${command}'`);
        try {
            const { stdout } = await pexec(command);
            response.result = stdout;
        } catch (error) {
            response.error = `Executing command: ${error}`;
        }
    } else {
        let formattedPayload = handler === 'cprintln' || handler === 'cprint' ? processMessageColors(payload) : payload;

        const timestamp = new Date().toLocaleTimeString();
        const consoleEntries = formattedPayload.split('\n').map(entry => `[${timestamp}] ${entry}`);

        // Create a new line for every payload received
        if (!consoleData[deviceId]) {
            consoleData[deviceId] = [];
        }
        consoleData[deviceId].push(...consoleEntries);

        response.message = formattedPayload;
    }

    console.log("Payload received: ", payload); // Log the payload
    console.log("Response being sent: ", response); // Log the response

    return JSON.stringify(response);
}

function clearConsole(deviceId) {
    consoleData[deviceId] = [];
}

const server = http.createServer(async (req, res) => {
    const parsedUrl = url.parse(req.url, true);
    console.log(`Received request: ${req.method} ${req.url}`);

    if (req.method === 'GET' && parsedUrl.pathname === '/') {
        fs.readFile(path.join(__dirname, 'console.html'), (err, data) => {
            if (err) {
                res.writeHead(500, { 'Content-Type': 'text/plain' });
                res.end('500 - Internal Server Error');
            } else {
                res.writeHead(200, { 'Content-Type': 'text/html' });
                res.end(data);
            }
        });
    } else if (req.method === 'POST' && parsedUrl.pathname === pathEndpoint) {
        let payload = '';
        req.on('data', (chunk) => {
            payload += chunk;
        });
        req.on('end', async () => {
            console.log(`Received POST data: ${payload}`);
            if (payload) {
                try {
                    const data = JSON.parse(payload);
                    deviceLastContact[data.deviceId] = Date.now(); // Update last contact time
                    const response = await handlePayload(data.payload, data.handler, data.deviceId);
                    res.writeHead(200, { 'Content-Type': 'application/json' });
                    res.end(response);
                } catch (e) {
                    res.writeHead(400, { 'Content-Type': 'application/json' });
                    res.end(`{"error": "Invalid JSON in request: ${e}"}`);
                }
            } else {
                res.writeHead(400, { 'Content-Type': 'application/json' });
                res.end('{"error": "Missing payload attribute in POST body"}');
            }
        });
    } else if (req.method === 'GET' && parsedUrl.pathname === '/console') {
        res.writeHead(200, { 'Content-Type': 'application/json' });
        res.end(JSON.stringify({ consoleData, deviceLastContact }));
    } else if (req.method === 'POST' && parsedUrl.pathname === '/clear') {
        let body = '';
        req.on('data', (chunk) => {
            body += chunk;
        });
        req.on('end', () => {
            const { deviceId } = JSON.parse(body);
            clearConsole(deviceId);
            res.writeHead(200, { 'Content-Type': 'application/json' });
            res.end(`{"message": "Console for ${deviceId} cleared"}`);
        });
    } else if (req.method === 'POST' && parsedUrl.pathname === '/command') {
        let body = '';
        req.on('data', (chunk) => {
            body += chunk;
        });
        req.on('end', () => {
            const { deviceId, command } = JSON.parse(body);
            console.log(`Received command for ${deviceId}: ${command}`);
            if (!commandQueue[deviceId]) {
                commandQueue[deviceId] = [];
            }
            commandQueue[deviceId].push(command);
            res.writeHead(200, { 'Content-Type': 'application/json' });
            res.end(`{"message": "Command queued for ${deviceId}"}`);
        });
    } else if (req.method === 'GET' && parsedUrl.pathname === '/KlingShell/commands') {
        const { deviceId } = parsedUrl.query;
        console.log(`Checking for commands for ${deviceId}`);
        deviceLastContact[deviceId] = Date.now(); // Update last contact time
        if (deviceId && commandQueue[deviceId] && commandQueue[deviceId].length > 0) {
            res.writeHead(200, { 'Content-Type': 'application/json' });
            res.end(JSON.stringify({ commands: commandQueue[deviceId] }));
            commandQueue[deviceId] = []; // Clear commands after sending
        } else {
            res.writeHead(200, { 'Content-Type': 'application/json' });
            res.end(JSON.stringify({ commands: [] }));
        }
    } else if (req.method === 'POST' && parsedUrl.pathname === '/remove') {
        let body = '';
        req.on('data', (chunk) => {
            body += chunk;
        });
        req.on('end', () => {
            const { deviceId } = JSON.parse(body);
            console.log(`Removing device ${deviceId}`);
            delete deviceLastContact[deviceId];
            delete consoleData[deviceId];
            res.writeHead(200, { 'Content-Type': 'application/json' });
            res.end('{"message": "Device removed"}');
        });
    } else if (req.method === 'GET' && parsedUrl.pathname.startsWith('/sounds/')) {
        // Serve the WAV file
        const soundFile = path.join(__dirname, parsedUrl.pathname);
        fs.exists(soundFile, (exists) => {
            if (exists) {
                res.writeHead(200, { 'Content-Type': 'audio/wav' });
                const readStream = fs.createReadStream(soundFile);
                readStream.pipe(res);
            } else {
                res.writeHead(404, { 'Content-Type': 'text/plain' });
                res.end('404 - File Not Found');
            }
        });
    } else {
        res.writeHead(404, { 'Content-Type': 'application/json' });
        res.end('{"error": "Route not found"}');
    }
});

server.listen(port, () => {
    console.log(`Server is listening on http://localhost:${port}${pathEndpoint}`);
});
