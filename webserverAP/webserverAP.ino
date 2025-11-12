#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>

/* Configuración WiFi */
const char* ssid = "Parqueo JJ";
const char* password = "12345678";
IPAddress local_ip(192,168,4,1);
IPAddress gateway(192,168,4,1);
IPAddress subnet(255,255,255,0);

/* Configuración I2C ESCLAVO */
#define I2C_SDA 21
#define I2C_SCL 22
#define ESP32_SLAVE_ADDRESS 0x28  // Dirección del ESP32 como esclavo

WebServer server(80);

/* Estado de los parqueos: 0 = ocupado, 1 = libre */
uint8_t parkingStatus[8] = {0, 0, 0, 0, 0, 0, 0, 0};
int espaciosLibres = 0;
unsigned long lastUpdate = 0;

/* Buffer I2C para recibir datos del maestro STM32 */
volatile bool newDataReceived = false;

void setup() {
  Serial.begin(115200);
  Serial.println("\n\n========================================");
  Serial.println("   Sistema de Parqueo - ESP32 ESCLAVO");
  Serial.println("========================================\n");

  // Inicializar I2C como ESCLAVO
  Wire.begin(ESP32_SLAVE_ADDRESS, I2C_SDA, I2C_SCL, 100000);
  Wire.onReceive(receiveEvent);  // Callback cuando el maestro envía datos
  Wire.onRequest(requestEvent);  // Callback cuando el maestro solicita datos
  
  Serial.println("I2C Esclavo inicializado");
  Serial.printf("  SDA: GPIO%d\n", I2C_SDA);
  Serial.printf("  SCL: GPIO%d\n", I2C_SCL);
  Serial.printf("  Direccion esclavo: 0x%02X\n\n", ESP32_SLAVE_ADDRESS);

  // Configurar WiFi como Access Point
  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(local_ip, gateway, subnet);
  WiFi.softAP(ssid, password);
  
  delay(500);
  
  Serial.println("WiFi Access Point iniciado");
  Serial.printf("  SSID: %s\n", ssid);
  Serial.printf("  Password: %s\n", password);
  Serial.printf("  IP: %s\n\n", WiFi.softAPIP().toString().c_str());

  // Configurar servidor web
  server.on("/", handle_OnConnect);
  server.on("/status", handle_Status);
  server.onNotFound(handle_NotFound);

  server.begin();
  Serial.println("Servidor web iniciado");
  Serial.println("Esperando datos del maestro STM32...");
  Serial.println("========================================\n");
}

void loop() {
  server.handleClient();
  
  // Procesar datos recibidos si hay nuevos
  if (newDataReceived) {
    newDataReceived = false;
    Serial.printf("[I2C RX] Espacios: [%d,%d,%d,%d,%d,%d,%d,%d] | Libres: %d/8\n",
                  parkingStatus[0], parkingStatus[1], parkingStatus[2], parkingStatus[3],
                  parkingStatus[4], parkingStatus[5], parkingStatus[6], parkingStatus[7],
                  espaciosLibres);
  }
}

/* 
 * Callback I2C: Se ejecuta cuando el maestro STM32 ENVÍA datos al ESP32
 * Formato esperado: 9 bytes
 * - Byte 0: Total de espacios libres (0-8)
 * - Bytes 1-8: Estado de cada parqueo (0=ocupado, 1=libre)
 */
void receiveEvent(int numBytes) {
  if (numBytes >= 9) {
    // Leer byte 0: total de espacios libres
    espaciosLibres = Wire.read();
    
    // Leer bytes 1-8: estado individual de cada parqueo
    for (int i = 0; i < 8; i++) {
      if (Wire.available()) {
        parkingStatus[i] = Wire.read();
      }
    }
    
    // Limpiar bytes restantes si los hay
    while (Wire.available()) {
      Wire.read();
    }
    
    newDataReceived = true;
    
  } else {
    // Datos incompletos, limpiar buffer
    while (Wire.available()) {
      Wire.read();
    }
    Serial.printf("[I2C ERROR] Se esperaban 9 bytes, se recibieron %d\n", numBytes);
  }
}

/* 
 * Callback I2C: Se ejecuta cuando el maestro STM32 SOLICITA datos del ESP32
 * Por ahora solo enviamos un ACK o confirmación
 */
void requestEvent() {
  // El maestro puede solicitar confirmación de recepción
  uint8_t response = 0xAA;  // Byte de confirmación
  Wire.write(response);
}

void handle_OnConnect() {
  server.send(200, "text/html", SendHTML());
}

void handle_Status() {
  // Devuelve el estado de los 8 espacios en JSON
  String json = "{\"libres\":" + String(espaciosLibres) + 
                ",\"espacios\":[" + 
                String(parkingStatus[0]) + "," +
                String(parkingStatus[1]) + "," +
                String(parkingStatus[2]) + "," +
                String(parkingStatus[3]) + "," +
                String(parkingStatus[4]) + "," +
                String(parkingStatus[5]) + "," +
                String(parkingStatus[6]) + "," +
                String(parkingStatus[7]) + "]}";
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
  <title>Parqueo J&J | Estado en Tiempo Real</title>
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <style>
    * {
      margin: 0;
      padding: 0;
      box-sizing: border-box;
    }
    
    body {
      font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
      background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
      min-height: 100vh;
      padding: 20px;
    }
    
    header {
      text-align: center;
      color: white;
      margin-bottom: 30px;
    }
    
    header h1 {
      font-size: 2.5em;
      margin-bottom: 10px;
      text-shadow: 2px 2px 4px rgba(0,0,0,0.3);
    }
    
    header p {
      font-size: 1.1em;
      opacity: 0.9;
    }
    
    .dashboard {
      max-width: 1200px;
      margin: 0 auto;
      display: grid;
      grid-template-columns: 2fr 1fr;
      gap: 20px;
    }
    
    .parking-section, .available-section {
      background: white;
      border-radius: 15px;
      padding: 25px;
      box-shadow: 0 10px 30px rgba(0,0,0,0.2);
    }
    
    .status-label {
      font-size: 1.3em;
      font-weight: bold;
      color: #333;
      margin-bottom: 20px;
      text-align: center;
    }
    
    .parking-lot {
      display: flex;
      flex-direction: column;
      gap: 15px;
    }
    
    .parking-row {
      display: grid;
      grid-template-columns: repeat(4, 1fr);
      gap: 10px;
    }
    
    .spot-card {
      background: #f8f9fa;
      border-radius: 10px;
      padding: 15px;
      text-align: center;
      transition: all 0.3s ease;
      border: 3px solid transparent;
    }
    
    .spot-card.free {
      border-color: #28a745;
      background: #d4edda;
    }
    
    .spot-card.occupied {
      border-color: #dc3545;
      background: #f8d7da;
    }
    
    .spot-card.disabled {
      border-color: #6c757d;
      background: #e9ecef;
      opacity: 0.6;
    }
    
    .spot-label {
      font-weight: bold;
      font-size: 0.9em;
      color: #666;
      margin-bottom: 8px;
    }
    
    .car-icon {
      font-size: 2em;
      margin: 10px 0;
    }
    
    .indicator {
      display: flex;
      justify-content: center;
      gap: 8px;
      margin: 8px 0;
    }
    
    .led {
      width: 12px;
      height: 12px;
      border-radius: 50%;
      box-shadow: 0 0 5px rgba(0,0,0,0.3);
    }
    
    .led.green {
      background: #28a745;
    }
    
    .led.red {
      background: #dc3545;
    }
    
    .led.gray {
      background: #6c757d;
    }
    
    .spot-status {
      font-size: 0.85em;
      font-weight: bold;
      padding: 5px;
      border-radius: 5px;
      margin-top: 5px;
    }
    
    .spot-status.free {
      color: #155724;
      background: #c3e6cb;
    }
    
    .spot-status.occupied {
      color: #721c24;
      background: #f5c6cb;
    }
    
    .spot-status.disabled {
      color: #6c757d;
      background: #d6d8db;
    }

    /* ===== Calle central mejorada ===== */
    .street {
      margin: 16px 0;
      height: 72px; /* altura de la calle */
      background: linear-gradient(#3b3b3b, #2a2a2a); /* apariencia de asfalto */
      border-radius: 10px;
      box-shadow: inset 0 -6px 12px rgba(0,0,0,0.5);
      position: relative;
      overflow: hidden;
    }

    /* líneas laterales punteadas (tenues) */
    .street .side-line {
      position: absolute;
      left: 6px;
      right: 6px;
      height: 6px;
      top: 50%;
      transform: translateY(-50%);
      background: repeating-linear-gradient(
        to right,
        rgba(255,255,255,0.9) 0px,
        rgba(255,255,255,0.9) 6px,
        transparent 6px,
        transparent 20px
      );
      opacity: 0.12;
      border-radius: 4px;
      pointer-events: none;
    }

    /* línea central amarilla discontinua (más ancha, con brillo) */
    .street .center-line {
      position: absolute;
      left: 10%;
      width: 80%;
      height: 12px;
      top: 50%;
      transform: translateY(-50%);
      background: repeating-linear-gradient(
        to right,
        #ffc107 0px,
        #ffc107 28px,
        transparent 28px,
        transparent 56px
      );
      border-radius: 6px;
      box-shadow: 0 0 8px rgba(255,193,7,0.12);
      pointer-events: none;
    }

    /* sombra tenue sobre la calle para realismo */
    .street::after {
      content: "";
      position: absolute;
      left: 0;
      right: 0;
      top: 0;
      height: 20%;
      background: linear-gradient(rgba(0,0,0,0.12), transparent);
      pointer-events: none;
    }
    /* ===== fin calle ===== */

    .available-section {
      display: flex;
      flex-direction: column;
      justify-content: center;
      align-items: center;
      background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
      color: white;
    }
    
    .big-number {
      font-size: 5em;
      font-weight: bold;
      text-shadow: 3px 3px 6px rgba(0,0,0,0.3);
    }
    
    .available-label {
      font-size: 1.2em;
      text-align: center;
      margin-top: 10px;
      opacity: 0.95;
    }
    
    .info-note {
      margin-top: 20px;
      padding: 15px;
      background: #fff3cd;
      border-left: 4px solid #ffc107;
      border-radius: 5px;
      font-size: 0.9em;
      color: #856404;
    }
    
    @media (max-width: 768px) {
      .dashboard {
        grid-template-columns: 1fr;
      }
      
      .parking-row {
        grid-template-columns: repeat(2, 1fr);
      }
      
      header h1 {
        font-size: 1.8em;
      }

      .street { height: 48px; }
      .street .center-line { height: 8px; }
      .street .side-line { height: 4px; }
    }
  </style>
</head>
<body>
  <header>
    <h1>🚗 Parqueo J&J</h1>
    <p>Monitoreo en tiempo real</p>
  </header>
  <div class="dashboard">
    <div class="parking-section">
      <div class="status-label">Estado de Parqueos</div>
      <div class="parking-lot" id="parking-lot"></div>
      <div class="info-note">
        ℹ️ <strong>Sistema de 8 espacios:</strong><br>
        Datos recibidos del controlador maestro STM32
      </div>
    </div>
    <div class="available-section">
      <div class="big-number" id="available-count">-</div>
      <div class="available-label">Disponibles<br>
    </div>
  </div>
  <script>
    let carIcons = ['🚗','🚙','🚕','🚔','🚘','🚖','🚓','🎏'];
    
    function renderParkingLot(data) {
      const lot = document.getElementById('parking-lot');
      lot.innerHTML = '';
      
      // Primera fila (espacios 1-4)
      const row1 = document.createElement('div');
      row1.className = 'parking-row';
      for (let i = 0; i < 4; i++) {
        row1.appendChild(createSpotCard(i, data.espacios[i]));
      }
      lot.appendChild(row1);
      
      // Calle (ahora con diseño realista)
      const street = document.createElement('div');
      street.className = 'street';
      street.innerHTML = '<div class="side-line"></div><div class="center-line"></div>';
      lot.appendChild(street);
      
      // Segunda fila (espacios 5-8)
      const row2 = document.createElement('div');
      row2.className = 'parking-row';
      for (let i = 4; i < 8; i++) {
        row2.appendChild(createSpotCard(i, data.espacios[i]));
      }
      lot.appendChild(row2);
      
      document.getElementById('available-count').textContent = data.libres;
    }
    
    function createSpotCard(index, libre) {
      const div = document.createElement('div');
      const occupied = (libre === 0);
      
      div.className = 'spot-card ' + (occupied ? 'occupied' : 'free');
      div.innerHTML = `
        <div class="spot-label">P${index+1}</div>
        <div class="car-icon" style="opacity:${occupied ? 1 : 0.3}">${carIcons[index]}</div>
        <div class="indicator">
          <div class="led green" style="opacity:${occupied ? 0.2 : 1}"></div>
          <div class="led red" style="opacity:${occupied ? 1 : 0.2}"></div>
        </div>
        <div class="spot-status ${occupied ? 'occupied' : 'free'}">${occupied ? '✗ OCUPADO' : '✓ LIBRE'}</div>
      `;
      
      return div;
    }
    
    // Actualizar estado cada 500ms
    function fetchStatus() {
      fetch('/status')
        .then(response => response.json())
        .then(data => renderParkingLot(data))
        .catch(err => console.error('Error:', err));
    }
    
    setInterval(fetchStatus, 500);
    window.onload = fetchStatus;
  </script>
</body>
</html>
)rawliteral";
  return ptr;
}