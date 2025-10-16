// One-port HTTP + WebSocket signaling + static client
// Run: PORT=8080 node server.js
const http = require('http');
const fs = require('fs');
const path = require('path');
const WebSocket = require('ws');

const PORT = process.env.PORT || 8080;
const publicDir = path.join(__dirname, 'public');

const server = http.createServer((req, res) => {
  // Serve index.html and static files
  let filePath = req.url === '/' ? path.join(publicDir, 'index.html') : path.join(publicDir, req.url);
  // Prevent directory traversal
  if (!filePath.startsWith(publicDir)) {
    res.writeHead(403); return res.end('Forbidden');
  }
  fs.readFile(filePath, (err, data) => {
    if (err) {
      res.writeHead(404); return res.end('Not found');
    }
    // Basic content-type mapping
    const ext = path.extname(filePath);
    const contentType = ({
      '.html': 'text/html',
      '.js': 'application/javascript',
      '.css': 'text/css',
      '.map': 'application/json'
    })[ext] || 'application/octet-stream';
    res.writeHead(200, { 'Content-Type': contentType });
    res.end(data);
  });
});

const wss = new WebSocket.Server({ server });

wss.on('connection', (ws) => {
  console.log('WS client connected, total:', wss.clients.size);
  ws.on('message', (msg) => {
    // Simple relay to others
    wss.clients.forEach((client) => {
      if (client !== ws && client.readyState === WebSocket.OPEN) {
        client.send(msg);
      }
    });
  });
  ws.on('close', () => console.log('WS client disconnected'));
});

server.listen(PORT, '0.0.0.0', () => {
  console.log(`HTTP+WS signaling server on http://0.0.0.0:${PORT}`);
  console.log(`Open from phone browser: http://<UBUNTU_IP>:${PORT}/`);
});