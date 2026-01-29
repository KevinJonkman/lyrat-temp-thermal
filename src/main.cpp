// ESP32 LyraT - Dual DS18B20 + MLX90640 Thermal Camera
// Reads 2x DS18B20 temp sensors + MLX90640 thermal array
// Serves data via web interface on WiFi

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Wire.h>
#include <Adafruit_MLX90640.h>

// ============== PIN DEFINITIONS (ESP32 LyraT) ==============
#define ONE_WIRE_BUS    13   // DS18B20 data pin (both sensors on same bus)
#define MLX_SDA_PIN     15   // MLX90640 I2C SDA (Wire1)
#define MLX_SCL_PIN     14   // MLX90640 I2C SCL (Wire1)
#define BLUE_LED_PIN    22   // Blue LED on LyraT

// ============== WiFi ==============
const char* WIFI_SSID = "BTAC Medewerkers";
const char* WIFI_PASS = "Next3600$!";

// ============== OBJECTS ==============
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature dsSensors(&oneWire);
Adafruit_MLX90640 mlx;
WebServer server(80);

// ============== DATA ==============
#define MLX_COLS 32
#define MLX_ROWS 24
#define MLX_PIXELS (MLX_COLS * MLX_ROWS)

float mlxFrame[MLX_PIXELS];
float dsTemp1 = -127.0;
float dsTemp2 = -127.0;
DeviceAddress dsAddr1, dsAddr2;
int dsCount = 0;
bool mlxConnected = false;
float mlxMax = 0, mlxMin = 999, mlxAvg = 0;
unsigned long lastDsRequest = 0;
bool dsConversionRequested = false;
unsigned long lastMlxRead = 0;

// ============== DS18B20 SETUP ==============
void setupDS18B20() {
  dsSensors.begin();
  dsCount = dsSensors.getDeviceCount();
  Serial.printf("[DS18B20] Found %d sensor(s)\n", dsCount);

  if (dsCount >= 1) {
    dsSensors.getAddress(dsAddr1, 0);
    Serial.printf("[DS18B20] Sensor 1: %02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X\n",
      dsAddr1[0], dsAddr1[1], dsAddr1[2], dsAddr1[3],
      dsAddr1[4], dsAddr1[5], dsAddr1[6], dsAddr1[7]);
  }
  if (dsCount >= 2) {
    dsSensors.getAddress(dsAddr2, 1);
    Serial.printf("[DS18B20] Sensor 2: %02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X\n",
      dsAddr2[0], dsAddr2[1], dsAddr2[2], dsAddr2[3],
      dsAddr2[4], dsAddr2[5], dsAddr2[6], dsAddr2[7]);
  }

  dsSensors.setResolution(12);
  dsSensors.setWaitForConversion(false);
  Serial.println("[DS18B20] Ready (12-bit, non-blocking)");
}

// ============== MLX90640 SETUP ==============
void setupMLX() {
  Wire1.begin(MLX_SDA_PIN, MLX_SCL_PIN, 400000);

  if (mlx.begin(MLX90640_I2CADDR_DEFAULT, &Wire1)) {
    mlxConnected = true;
    mlx.setMode(MLX90640_CHESS);
    mlx.setResolution(MLX90640_ADC_18BIT);
    mlx.setRefreshRate(MLX90640_4_HZ);
    Serial.println("[MLX] Initialized OK - 32x24 @ 4Hz");
  } else {
    Serial.println("[MLX] NOT FOUND - check wiring!");
  }
}

// ============== MLX READ ==============
void mlxRead() {
  if (!mlxConnected) return;
  if (millis() - lastMlxRead < 500) return;
  lastMlxRead = millis();

  if (mlx.getFrame(mlxFrame) != 0) return;

  mlxMax = -40;
  mlxMin = 300;
  float sum = 0;
  int validCount = 0;

  for (int i = 0; i < MLX_PIXELS; i++) {
    float t = mlxFrame[i];
    if (t > -20 && t < 200) {
      if (t > mlxMax) mlxMax = t;
      if (t < mlxMin) mlxMin = t;
      sum += t;
      validCount++;
    }
  }
  if (validCount > 0) mlxAvg = sum / validCount;
}

// ============== DS18B20 NON-BLOCKING READ ==============
void dsRequestTemps() {
  dsSensors.requestTemperatures();
  dsConversionRequested = true;
  lastDsRequest = millis();
}

void dsReadResults() {
  if (!dsConversionRequested) return;
  if (millis() - lastDsRequest < 800) return;
  dsConversionRequested = false;

  if (dsCount >= 1) {
    float t = dsSensors.getTempC(dsAddr1);
    if (t > -50 && t < 125 && t != 85.0) dsTemp1 = t;
  }
  if (dsCount >= 2) {
    float t = dsSensors.getTempC(dsAddr2);
    if (t > -50 && t < 125 && t != 85.0) dsTemp2 = t;
  }
}

// ============== WEB: MAIN PAGE ==============
void handleRoot() {
  String h = "<!DOCTYPE html><html><head><meta charset='UTF-8'>";
  h += "<meta name='viewport' content='width=device-width,initial-scale=1'>";
  h += "<title>LyraT Sensor Hub</title>";
  h += "<style>";
  h += "body{background:#111;color:#eee;font-family:monospace;margin:0;padding:10px}";
  h += "h1{color:#0af;text-align:center}";
  h += ".panel{background:#1a1a2e;border-radius:8px;padding:15px;margin:10px 0}";
  h += ".panel h2{margin:0 0 10px;color:#0cf;font-size:1.1em}";
  h += ".row{display:flex;justify-content:space-between;padding:5px 0;border-bottom:1px solid #333}";
  h += ".row .k{color:#888}.row .v{color:#0f0;font-weight:bold}";
  h += ".thermal{text-align:center;margin:10px 0}";
  h += "canvas{border:1px solid #333;border-radius:4px;max-width:100%}";
  h += ".warn{color:#f80}.err{color:#f44}.ok{color:#0f0}";
  h += "</style></head><body>";
  h += "<h1>LyraT Sensor Hub</h1>";

  // DS18B20 Panel
  h += "<div class='panel'><h2>DS18B20 Temperature</h2>";
  h += "<div class='row'><span class='k'>Sensor 1</span><span class='v' id='t1'>--</span></div>";
  h += "<div class='row'><span class='k'>Sensor 2</span><span class='v' id='t2'>--</span></div>";
  h += "<div class='row'><span class='k'>Sensors found</span><span class='v' id='cnt'>--</span></div>";
  h += "</div>";

  // MLX90640 Panel
  h += "<div class='panel'><h2>MLX90640 Thermal Camera</h2>";
  h += "<div class='row'><span class='k'>Max Temp</span><span class='v' id='tmax'>--</span></div>";
  h += "<div class='row'><span class='k'>Min Temp</span><span class='v' id='tmin'>--</span></div>";
  h += "<div class='row'><span class='k'>Avg Temp</span><span class='v' id='tavg'>--</span></div>";
  h += "<div class='row'><span class='k'>Status</span><span class='v' id='mlxst'>--</span></div>";
  h += "</div>";

  // Thermal Image
  h += "<div class='panel'><h2>Thermal View</h2>";
  h += "<div class='thermal'><canvas id='cv' width='320' height='240'></canvas></div>";
  h += "</div>";

  // JavaScript
  h += "<script>";
  h += "var $=function(id){return document.getElementById(id);};";

  // Color map (blue->green->yellow->red->white)
  h += "function tempColor(t,mn,mx){";
  h += "var r=0,g=0,b=0,f=(t-mn)/(mx-mn||1);f=Math.max(0,Math.min(1,f));";
  h += "if(f<0.25){b=255;g=Math.round(f*4*255);}";
  h += "else if(f<0.5){g=255;b=Math.round((0.5-f)*4*255);}";
  h += "else if(f<0.75){g=255;r=Math.round((f-0.5)*4*255);}";
  h += "else{r=255;g=Math.round((1-f)*4*255);}";
  h += "return 'rgb('+r+','+g+','+b+')';}";

  // Status update
  h += "function upd(){fetch('/status').then(r=>r.json()).then(d=>{";
  h += "$('t1').innerText=d.t1>-100?d.t1.toFixed(2)+' C':'N/C';";
  h += "$('t2').innerText=d.t2>-100?d.t2.toFixed(2)+' C':'N/C';";
  h += "$('cnt').innerText=d.dsCount;";
  h += "$('tmax').innerText=d.mlxMax.toFixed(1)+' C';";
  h += "$('tmin').innerText=d.mlxMin.toFixed(1)+' C';";
  h += "$('tavg').innerText=d.mlxAvg.toFixed(1)+' C';";
  h += "$('mlxst').innerText=d.mlxOk?'Connected':'NOT FOUND';";
  h += "$('mlxst').className='v '+(d.mlxOk?'ok':'err');";
  h += "}).catch(()=>{});}";

  // Thermal image update
  h += "function updThermal(){fetch('/thermaldata').then(r=>r.json()).then(d=>{";
  h += "var cv=$('cv'),ctx=cv.getContext('2d');";
  h += "var pw=cv.width/32,ph=cv.height/24;";
  h += "for(var y=0;y<24;y++)for(var x=0;x<32;x++){";
  h += "ctx.fillStyle=tempColor(d.pixels[y*32+x],d.min,d.max);";
  h += "ctx.fillRect(x*pw,y*ph,pw,ph);}";
  h += "}).catch(()=>{});}";

  h += "setInterval(upd,2000);setInterval(updThermal,1000);upd();updThermal();";
  h += "</script></body></html>";

  server.send(200, "text/html", h);
}

// ============== WEB: STATUS JSON ==============
void handleStatus() {
  String j = "{";
  j += "\"t1\":" + String(dsTemp1, 2);
  j += ",\"t2\":" + String(dsTemp2, 2);
  j += ",\"dsCount\":" + String(dsCount);
  j += ",\"mlxOk\":" + String(mlxConnected ? "true" : "false");
  j += ",\"mlxMax\":" + String(mlxMax, 1);
  j += ",\"mlxMin\":" + String(mlxMin, 1);
  j += ",\"mlxAvg\":" + String(mlxAvg, 1);
  j += "}";
  server.send(200, "application/json", j);
}

// ============== WEB: THERMAL DATA JSON ==============
void handleThermalData() {
  if (!mlxConnected) {
    server.send(200, "application/json", "{\"ok\":false}");
    return;
  }

  String j = "{\"ok\":true,\"min\":";
  j += String(mlxMin, 1);
  j += ",\"max\":";
  j += String(mlxMax, 1);
  j += ",\"pixels\":[";

  for (int i = 0; i < MLX_PIXELS; i++) {
    if (i > 0) j += ",";
    j += String(mlxFrame[i], 1);
  }
  j += "]}";

  server.send(200, "application/json", j);
}

// ============== WEB: RESCAN DS18B20 ==============
void handleRescan() {
  Serial.println("\n[DS18B20] === BUS SCAN ===");

  // First: try multiple GPIO pins to find where sensors are
  int tryPins[] = {13, 4, 2, 27, 32, 33, 19, 12};
  int numTryPins = 8;
  Serial.println("[DS18B20] Scanning multiple pins...");
  for (int p = 0; p < numTryPins; p++) {
    OneWire testBus(tryPins[p]);
    byte testAddr[8];
    testBus.reset_search();
    delay(100);
    int cnt = 0;
    while (testBus.search(testAddr)) cnt++;
    Serial.printf("[DS18B20] GPIO %d: %d device(s)\n", tryPins[p], cnt);
  }

  // Now scan the configured pin
  Serial.printf("[DS18B20] Detailed scan on GPIO %d:\n", ONE_WIRE_BUS);

  // Raw OneWire search
  byte addr[8];
  int found = 0;
  String j = "{\"addresses\":[";

  oneWire.reset_search();
  delay(250);

  // Test if bus pulls high (pullup present)
  pinMode(ONE_WIRE_BUS, INPUT);
  delay(10);
  int pinState = digitalRead(ONE_WIRE_BUS);
  Serial.printf("[DS18B20] Pin %d idle state: %s (needs HIGH for pullup)\n", ONE_WIRE_BUS, pinState ? "HIGH" : "LOW");

  oneWire.reset_search();
  delay(250);

  while (oneWire.search(addr)) {
    if (found > 0) j += ",";
    j += "\"";
    for (int i = 0; i < 8; i++) {
      if (i > 0) j += ":";
      char hex[3];
      snprintf(hex, sizeof(hex), "%02X", addr[i]);
      j += hex;
    }
    j += "\"";

    Serial.printf("[DS18B20] Device %d: %02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X",
      found, addr[0], addr[1], addr[2], addr[3], addr[4], addr[5], addr[6], addr[7]);

    if (OneWire::crc8(addr, 7) != addr[7]) {
      Serial.println(" - CRC ERROR!");
    } else if (addr[0] == 0x28) {
      Serial.println(" - DS18B20");
    } else if (addr[0] == 0x10) {
      Serial.println(" - DS18S20");
    } else {
      Serial.printf(" - Unknown family 0x%02X\n", addr[0]);
    }
    found++;
  }

  Serial.printf("[DS18B20] Scan complete: %d device(s) found\n", found);

  // Re-init DallasTemperature
  dsSensors.begin();
  dsCount = dsSensors.getDeviceCount();
  Serial.printf("[DS18B20] DallasTemperature sees %d sensor(s)\n", dsCount);

  if (dsCount >= 1) {
    dsSensors.getAddress(dsAddr1, 0);
    Serial.printf("[DS18B20] Sensor 1: %02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X\n",
      dsAddr1[0], dsAddr1[1], dsAddr1[2], dsAddr1[3],
      dsAddr1[4], dsAddr1[5], dsAddr1[6], dsAddr1[7]);
  }
  if (dsCount >= 2) {
    dsSensors.getAddress(dsAddr2, 1);
    Serial.printf("[DS18B20] Sensor 2: %02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X\n",
      dsAddr2[0], dsAddr2[1], dsAddr2[2], dsAddr2[3],
      dsAddr2[4], dsAddr2[5], dsAddr2[6], dsAddr2[7]);
  }

  dsSensors.setResolution(12);
  dsSensors.setWaitForConversion(false);

  j += "],\"rawFound\":" + String(found);
  j += ",\"dsCount\":" + String(dsCount);
  j += ",\"pin\":" + String(ONE_WIRE_BUS);
  j += ",\"pinState\":\"" + String(pinState ? "HIGH" : "LOW") + "\"";
  j += "}";

  server.send(200, "application/json", j);
}

// ============== SETUP ==============
void setup() {
  Serial.begin(115200);
  delay(500);

  Serial.println("\n========== LyraT Sensor Hub ==========\n");

  pinMode(BLUE_LED_PIN, OUTPUT);
  digitalWrite(BLUE_LED_PIN, LOW);

  // DS18B20
  setupDS18B20();

  // MLX90640
  setupMLX();

  // WiFi
  Serial.printf("[WIFI] Connecting to %s", WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.printf("\n[WIFI] %s\n", WiFi.localIP().toString().c_str());

  // Web server routes
  server.on("/", handleRoot);
  server.on("/status", handleStatus);
  server.on("/thermaldata", handleThermalData);
  server.on("/rescan", handleRescan);
  server.begin();

  // Initial temp request
  dsRequestTemps();

  digitalWrite(BLUE_LED_PIN, HIGH);
  Serial.printf("\nReady: http://%s\n", WiFi.localIP().toString().c_str());
}

// ============== LOOP ==============
void loop() {
  server.handleClient();
  yield();

  // MLX read (~250ms block at 4Hz, every 500ms)
  if (mlxConnected) {
    server.handleClient();
    mlxRead();
    server.handleClient();
    yield();
  }

  // DS18B20 non-blocking read
  dsReadResults();

  // Request new temps every 2 seconds
  static unsigned long lastTempRequest = 0;
  if (millis() - lastTempRequest > 2000) {
    lastTempRequest = millis();
    dsRequestTemps();
  }

  // LED heartbeat
  static unsigned long lastBlink = 0;
  if (millis() - lastBlink > 1000) {
    lastBlink = millis();
    digitalWrite(BLUE_LED_PIN, !digitalRead(BLUE_LED_PIN));
  }

  delay(2);
  yield();
}
