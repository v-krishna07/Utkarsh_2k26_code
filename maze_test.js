// Run: node maze_test.js   (no npm install needed, only built-in modules)
// First connect the laptop WiFi to the ESP32 hotspot "MazeBot".
const dgram = require('dgram');

const ESP_IP = '192.168.4.1';
const PORT = 4210;
const TH = 10;                       // cm threshold

const sock = dgram.createSocket('udp4');
const toCm = us => (us === 0 ? Infinity : (us * 0.0343) / 2);   // 0 = no echo = far
const show = d => (d === Infinity ? ' >100' : d.toFixed(0).padStart(5));

function decide(L, F, R) {
  if (F >= TH) return 'STRAIGHT (front clear)';
  const lOpen = L >= TH, rOpen = R >= TH;
  if (lOpen && rOpen) return L > R ? 'LEFT (more space)' : 'RIGHT (more space)';
  if (lOpen) return 'LEFT';
  if (rOpen) return 'RIGHT';
  return 'NONE (blocked everywhere)';
}

let count = 0, hz = 0;
setInterval(() => { hz = count * 2; count = 0; }, 500);

sock.on('message', m => {
  if (m.length !== 7) return;
  count++;
  const L = toCm(m.readUInt16LE(1));
  const F = toCm(m.readUInt16LE(3));
  const R = toCm(m.readUInt16LE(5));
  process.stdout.write(`\rL${show(L)}  F${show(F)}  R${show(R)} cm | ${String(hz).padStart(2)} Hz | ${decide(L, F, R).padEnd(28)}`);
});

sock.bind(PORT, () => {
  // hello/keepalive: tells the ESP32 where to send data
  setInterval(() => sock.send('H', PORT, ESP_IP), 500);
  console.log('Waiting for ESP32 at ' + ESP_IP + ' ... (is the laptop on the MazeBot WiFi?)');
});
