#include <WiFi.h>
#include <WebServer.h>

/* Configuración WiFi */
const char* ssid = "Parqueo JJ";
const char* password = "12345678";
IPAddress local_ip(192,168,4,1);
IPAddress gateway(192,168,1,1);
IPAddress subnet(255,255,255,0);

WebServer server(80);

/* Estado de los parqueos: 0 = libre, 1 = ocupado */
uint8_t parkingStatus[8] = {0, 1, 0, 1, 0, 0, 1, 0}; // Ejemplo inicial

void setup() {
  Serial.begin(115200);

  WiFi.softAP(ssid, password);
  WiFi.softAPConfig(local_ip, gateway, subnet);
  delay(100);

  server.on("/", handle_OnConnect);
  server.on("/status", handle_Status); // Endpoint para AJAX
  server.onNotFound(handle_NotFound);

  server.begin();
  Serial.println("HTTP server started");
}

void loop() {
  server.handleClient();
  // Aquí puedes actualizar parkingStatus[] según tus sensores reales
  // Ejemplo: parkingStatus[0] = digitalRead(sensorPin0);
}

void handle_OnConnect() {
  server.send(200, "text/html", SendHTML());
}

void handle_Status() {
  // Devuelve el estado de los parqueos en JSON
  String json = "[";
  for (int i = 0; i < 8; i++) {
    json += String(parkingStatus[i]);
    if (i < 7) json += ",";
  }
  json += "]";
  server.send(200, "application/json", json);
}

void handle_NotFound(){
  server.send(404, "text/plain", "Not found");
}

String SendHTML() {
  String ptr = R"rawliteral(
<!DOCTYPE html>
<html lang="es">
<head>
  <meta charset="UTF-8">
  <title>Parqueo-matic | Estado de Parqueo</title>
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <style>
    :root {
      --primary: #1a237e;
      --secondary: #3949ab;
      --accent: #00e676;
      --danger: #e53935;
      --bg: #f5f7fa;
      --card-bg: #fff;
      --shadow: 0 4px 16px #0002;
      --border-radius: 16px;
      --street: #b0bec5;
      --street-line: #fffde7;
      --spot-free: #00e676;
      --spot-occupied: #e53935;
    }
    body {
      margin: 0;
      background: var(--bg);
      font-family: 'Segoe UI', Arial, sans-serif;
      color: #222;
    }
    header {
      background: linear-gradient(90deg, var(--primary), var(--secondary));
      color: #fff;
      padding: 32px 0 24px 0;
      box-shadow: var(--shadow);
      text-align: center;
      border-bottom-left-radius: 32px;
      border-bottom-right-radius: 32px;
    }
    header h1 {
      margin: 0;
      font-size: 2.8rem;
      letter-spacing: 2px;
      font-weight: 700;
    }
    header p {
      margin: 8px 0 0 0;
      font-size: 1.2rem;
      opacity: 0.85;
    }
    .dashboard {
      max-width: 1100px;
      margin: 40px auto 0 auto;
      padding: 0 16px;
      display: flex;
      flex-direction: row;
      justify-content: center;
      align-items: center;
      gap: 40px;
    }
    .parking-section {
      background: var(--card-bg);
      border-radius: var(--border-radius);
      box-shadow: var(--shadow);
      padding: 32px 24px 24px 24px;
      display: flex;
      flex-direction: column;
      align-items: center;
      width: 100%;
      max-width: 700px;
    }
    .status-label {
      font-size: 1.2rem;
      color: #444;
      margin-bottom: 0;
      font-weight: 500;
    }
    .parking-lot {
      width: 100%;
      margin-top: 18px;
      display: flex;
      flex-direction: column;
      align-items: stretch;
      position: relative;
    }
    .parking-row {
      display: flex;
      justify-content: space-between;
      gap: 24px;
      z-index: 2;
    }
    .street {
      width: 100%;
      height: 48px;
      background: var(--street);
      margin: 18px 0 18px 0;
      border-radius: 12px;
      position: relative;
      box-shadow: 0 2px 8px #0001 inset;
      display: flex;
      align-items: center;
      justify-content: center;
      overflow: hidden;
    }
    .street-line {
      width: 98%;
      height: 6px;
      background: repeating-linear-gradient(
        to right,
        var(--street-line),
        var(--street-line) 24px,
        transparent 24px,
        transparent 40px
      );
      border-radius: 3px;
      opacity: 0.8;
    }
    .spot-card {
      background: #f8fafc;
      border-radius: 12px;
      box-shadow: 0 2px 8px #0001;
      padding: 18px 10px 14px 10px;
      display: flex;
      flex-direction: column;
      align-items: center;
      transition: box-shadow 0.2s, border 0.2s;
      border: 2px solid #e0e0e0;
      position: relative;
      min-width: 110px;
      min-height: 120px;
    }
    .spot-card.occupied {
      border-color: var(--spot-occupied);
      box-shadow: 0 2px 16px #e5393522;
    }
    .spot-card.free {
      border-color: var(--spot-free);
      box-shadow: 0 2px 16px #00e67622;
    }
    .car-icon {
      font-size: 2.5rem;
      margin-bottom: 8px;
      transition: opacity 0.3s;
    }
    .spot-label {
      font-size: 1.1rem;
      font-weight: 600;
      margin-bottom: 6px;
      color: #333;
    }
    .indicator {
      display: flex;
      gap: 8px;
      margin-bottom: 4px;
    }
    .led {
      width: 18px;
      height: 18px;
      border-radius: 50%;
      border: 2px solid #bbb;
      background: #eee;
      box-shadow: 0 0 0 #0000;
      transition: background 0.3s, box-shadow 0.3s;
    }
    .led.red {
      background: var(--spot-occupied);
      box-shadow: 0 0 12px #e5393555;
      border-color: var(--spot-occupied);
    }
    .led.green {
      background: var(--spot-free);
      box-shadow: 0 0 12px #00e67655;
      border-color: var(--spot-free);
    }
    .spot-status {
      font-size: 1rem;
      font-weight: 500;
      margin-top: 2px;
      color: #888;
      letter-spacing: 1px;
    }
    .available-section {
      display: flex;
      flex-direction: column;
      align-items: center;
      justify-content: center;
      min-width: 220px;
      background: var(--card-bg);
      border-radius: var(--border-radius);
      box-shadow: var(--shadow);
      padding: 32px 24px;
      height: 340px;
    }
    .big-number {
      font-size: 4.5rem;
      font-family: 'Consolas', monospace;
      color: var(--accent);
      margin-bottom: 8px;
      letter-spacing: 4px;
      text-shadow: 0 2px 8px #00e67644;
      background: #222;
      border-radius: 16px;
      padding: 10px 40px;
      box-shadow: 0 2px 16px #00e67622;
      display: inline-block;
    }
    .available-label {
      font-size: 1.2rem;
      color: #222;
      margin-top: 10px;
      font-weight: 500;
      letter-spacing: 1px;
    }
    @media (max-width: 1100px) {
      .dashboard {
        flex-direction: column;
        align-items: center;
        gap: 24px;
      }
      .available-section {
        flex-direction: row;
        width: 100%;
        min-width: unset;
        height: auto;
        margin-top: 24px;
      }
    }
    @media (max-width: 900px) {
      .parking-row {
        gap: 8px;
      }
      .parking-section {
        padding: 14px 4px;
      }
      .parking-lot {
        max-width: 98vw;
      }
      .spot-card {
        min-width: 70px;
        min-height: 80px;
        padding: 8px 2px 6px 2px;
      }
      .big-number {
        font-size: 2.8rem;
        padding: 6px 18px;
      }
      .available-section {
        padding: 18px 8px;
        height: auto;
      }
    }
  </style>
</head>
<body>
  <header>
    <h1>Parqueo-matic</h1>
    <p>Monitoreo en tiempo real de ocupación de parqueos</p>
  </header>
  <div class="dashboard">
    <div class="parking-section">
      <div class="status-label" style="margin-bottom: 12px;">Estado de Parqueos</div>
      <div class="parking-lot" id="parking-lot"></div>
    </div>
    <div class="available-section">
      <div class="big-number" id="available-count">8</div>
      <div class="available-label">Parqueos Disponibles</div>
    </div>
  </div>
  <script>
    let carIcons = ['🚗','🚙','🚕','🚓','🚘','🚖','🚔','🚗'];
    function renderParkingLot(statusArray) {
      const lot = document.getElementById('parking-lot');
      lot.innerHTML = '';
      const row1 = document.createElement('div');
      row1.className = 'parking-row';
      for (let i = 0; i < 4; i++) {
        row1.appendChild(createSpotCard(i, statusArray[i]));
      }
      lot.appendChild(row1);
      const street = document.createElement('div');
      street.className = 'street';
      street.innerHTML = '<div class="street-line"></div>';
      lot.appendChild(street);
      const row2 = document.createElement('div');
      row2.className = 'parking-row';
      for (let i = 4; i < 8; i++) {
        row2.appendChild(createSpotCard(i, statusArray[i]));
      }
      lot.appendChild(row2);
      let available = statusArray.filter(x => x === 0).length;
      document.getElementById('available-count').textContent = available;
    }
    function createSpotCard(index, occupied) {
      const div = document.createElement('div');
      div.className = 'spot-card ' + (occupied ? 'occupied' : 'free');
      div.innerHTML = `
        <div class="spot-label">P${index+1}</div>
        <div class="car-icon" style="opacity:${occupied ? 1 : 0.25}">${carIcons[index]}</div>
        <div class="indicator">
          <div class="led green" style="opacity:${occupied ? 0.2 : 1}"></div>
          <div class="led red" style="opacity:${occupied ? 1 : 0.2}"></div>
        </div>
        <div class="spot-status">${occupied ? 'Ocupado' : 'Libre'}</div>
      `;
      return div;
    }
    // Petición AJAX periódica para actualizar el estado
    function fetchStatus() {
      fetch('/status')
        .then(response => response.json())
        .then(data => renderParkingLot(data));
    }
    setInterval(fetchStatus, 2000); // Actualiza cada 2 segundos
    window.onload = fetchStatus;
  </script>
</body>
</html>
)rawliteral";
  return ptr;
}