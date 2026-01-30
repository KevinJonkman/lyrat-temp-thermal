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
#include <SPIFFS.h>
#include <FS.h>
#include <ArduinoOTA.h>

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

// ============== SPIFFS LOGGING ==============
bool spiffsReady = false;
bool loggingEnabled = false;
unsigned long logStartTime = 0;
unsigned long lastLogWrite = 0;
#define LOG_INTERVAL_MS 2000
#define LOG_FILE "/templog.csv"
#define MAX_LOG_SIZE 500000

// ============== DS18B20 SETUP ==============
void setupDS18B20() {
  // Diagnostic: test GPIO 13 state before and after pull-up
  Serial.printf("[DS18B20] GPIO %d raw read: %d\n", ONE_WIRE_BUS, digitalRead(ONE_WIRE_BUS));
  pinMode(ONE_WIRE_BUS, INPUT_PULLUP);
  delay(100);
  Serial.printf("[DS18B20] GPIO %d after INPUT_PULLUP: %d\n", ONE_WIRE_BUS, digitalRead(ONE_WIRE_BUS));

  // Try to drive HIGH manually to test if something is pulling LOW
  pinMode(ONE_WIRE_BUS, OUTPUT);
  digitalWrite(ONE_WIRE_BUS, HIGH);
  delay(10);
  Serial.printf("[DS18B20] GPIO %d after drive HIGH: %d\n", ONE_WIRE_BUS, digitalRead(ONE_WIRE_BUS));
  pinMode(ONE_WIRE_BUS, INPUT_PULLUP);
  delay(100);
  Serial.printf("[DS18B20] GPIO %d back to INPUT_PULLUP: %d\n", ONE_WIRE_BUS, digitalRead(ONE_WIRE_BUS));

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

// ============== SPIFFS INIT ==============
void initSPIFFS() {
  if (SPIFFS.begin(true)) {
    spiffsReady = true;
    Serial.printf("[SPIFFS] Mounted. Total: %u, Used: %u\n",
      SPIFFS.totalBytes(), SPIFFS.usedBytes());
  } else {
    Serial.println("[SPIFFS] Mount FAILED");
  }
}

// ============== TEMP LOGGING ==============
void startTempLog() {
  if (!spiffsReady) return;
  File f = SPIFFS.open(LOG_FILE, FILE_WRITE);
  if (f) {
    f.println("timestamp,t1,t2,mlx_max,mlx_avg");
    f.close();
    loggingEnabled = true;
    logStartTime = millis();
    lastLogWrite = 0;
    Serial.println("[LOG] Temp logging started");
  }
}

void appendTempLog() {
  if (!spiffsReady || !loggingEnabled) return;
  if (millis() - lastLogWrite < LOG_INTERVAL_MS) return;
  lastLogWrite = millis();

  File f = SPIFFS.open(LOG_FILE, FILE_APPEND);
  if (!f) return;

  if (f.size() > MAX_LOG_SIZE) {
    f.close();
    loggingEnabled = false;
    Serial.println("[LOG] Max size reached, logging stopped");
    return;
  }

  unsigned long elapsed = (millis() - logStartTime) / 1000;
  char line[80];
  snprintf(line, sizeof(line), "%lu,%.2f,%.2f,%.1f,%.1f",
    elapsed, dsTemp1, dsTemp2, mlxMax, mlxAvg);
  f.println(line);
  f.close();
}

// ============== WEB: LOG ENDPOINTS ==============
void handleStartLog() {
  startTempLog();
  server.send(200, "application/json", "{\"ok\":true,\"msg\":\"Logging started\"}");
}

void handleStopLog() {
  loggingEnabled = false;
  Serial.println("[LOG] Logging stopped");
  server.send(200, "application/json", "{\"ok\":true,\"msg\":\"Logging stopped\"}");
}

void handleDownload() {
  if (!spiffsReady || !SPIFFS.exists(LOG_FILE)) {
    server.send(404, "text/plain", "No log file");
    return;
  }
  File f = SPIFFS.open(LOG_FILE, FILE_READ);
  if (!f) {
    server.send(500, "text/plain", "Cannot open file");
    return;
  }
  server.streamFile(f, "text/csv");
  f.close();
}

void handleDeleteLog() {
  if (loggingEnabled) {
    loggingEnabled = false;
  }
  if (spiffsReady && SPIFFS.exists(LOG_FILE)) {
    SPIFFS.remove(LOG_FILE);
  }
  server.send(200, "application/json", "{\"ok\":true,\"msg\":\"Log deleted\"}");
}

void handleLogInfo() {
  size_t fileSize = 0;
  if (spiffsReady && SPIFFS.exists(LOG_FILE)) {
    File f = SPIFFS.open(LOG_FILE, FILE_READ);
    if (f) { fileSize = f.size(); f.close(); }
  }
  String j = "{\"logging\":" + String(loggingEnabled ? "true" : "false");
  j += ",\"size\":" + String(fileSize);
  j += ",\"totalSpace\":" + String(spiffsReady ? SPIFFS.totalBytes() : 0);
  j += ",\"usedSpace\":" + String(spiffsReady ? SPIFFS.usedBytes() : 0);
  j += ",\"freeSpace\":" + String(spiffsReady ? (SPIFFS.totalBytes() - SPIFFS.usedBytes()) : 0);
  j += "}";
  server.send(200, "application/json", j);
}

// ============== WEB: MAIN PAGE ==============
void handleRoot() {
  String h = "<!DOCTYPE html><html><head><meta charset='UTF-8'>";
  h += "<meta name='viewport' content='width=device-width,initial-scale=1'>";
  h += "<title>LyraT Sensor Hub v9.0</title>";
  h += "<script src='https://cdn.jsdelivr.net/npm/chart.js'></script>";
  h += "<link href='https://fonts.googleapis.com/css2?family=DSEG7+Classic:wght@400;700&display=swap' rel='stylesheet'>";
  h += "<style>";
  h += "@font-face{font-family:'DSEG7';src:url('https://cdn.jsdelivr.net/npm/dseg@0.46.0/fonts/DSEG7-Classic/DSEG7Classic-Bold.woff2')format('woff2')}";
  h += "*{box-sizing:border-box;margin:0;padding:0}";
  h += "body{font-family:sans-serif;background:#1a1a2e;color:#eee;padding:10px}";
  h += "h1{text-align:center;color:#0cf;margin-bottom:10px}";
  h += ".panel{background:#16213e;border-radius:6px;padding:12px;margin-bottom:10px}";
  h += ".panel h2{color:#0cf;font-size:0.9em;margin-bottom:8px}";
  // 7-segment display styles — identical to battery tester
  h += ".seg-display{display:flex;gap:20px;justify-content:center;margin-bottom:15px;flex-wrap:wrap}";
  h += ".seg-box{background:#0a0a15;border:2px solid #333;border-radius:8px;padding:15px 25px;text-align:center;min-width:180px}";
  h += ".seg-label{font-size:0.75em;color:#666;margin-bottom:5px;text-transform:uppercase}";
  h += ".seg-value{font-family:'DSEG7',monospace;font-size:3em;color:#0f0;text-shadow:0 0 10px #0f0}";
  h += ".seg-unit{font-size:0.4em;color:#888;margin-left:5px}";
  // Info row
  h += ".info-row{display:flex;justify-content:space-between;padding:4px 0;font-size:0.8em;color:#888}";
  h += ".info-row .v{color:#0f0;font-weight:bold}";
  // Chart
  h += ".graph-container{height:300px;position:relative}";
  h += ".tbtn{padding:6px 14px;border:1px solid #555;border-radius:4px;cursor:pointer;font-family:sans-serif;font-size:0.85em;margin:2px;background:#222;color:#aaa}";
  h += ".tbtn.active{background:#08c;color:#fff;border-color:#0af}";
  // Log panel
  h += ".row{display:flex;justify-content:space-between;font-size:0.85em;padding:3px 0}.row .k{color:#888}";
  h += ".row .v{color:#0f0;font-weight:bold}";
  h += "button{padding:8px 12px;border:none;border-radius:6px;font-weight:bold;cursor:pointer;margin:2px}";
  h += ".bg{background:#0a4;color:#fff}.br{background:#d33;color:#fff}.bb{background:#08c;color:#fff}.by{background:#c80;color:#fff}";
  h += "@media(max-width:900px){.seg-display{flex-direction:column;align-items:center}}";
  h += "</style></head><body>";
  h += "<h1>LyraT Sensor Hub v9.0</h1>";

  // === Row 1: Temperature displays ===
  h += "<div class='seg-display'>";
  h += "<div class='seg-box' style='border-color:#0a6'><div class='seg-label'>T1 (DS18B20)</div><div class='seg-value' id='sv1' style='color:#0f0;text-shadow:0 0 10px #0f0'>--.-<span class='seg-unit'>&deg;C</span></div></div>";
  h += "<div class='seg-box' style='border-color:#0a6'><div class='seg-label'>T2 (DS18B20)</div><div class='seg-value' id='sv2' style='color:#0ff;text-shadow:0 0 10px #0ff'>--.-<span class='seg-unit'>&deg;C</span></div></div>";
  h += "<div class='seg-box' style='border-color:#f44'><div class='seg-label'>MLX Max</div><div class='seg-value' id='sv3' style='color:#f44;text-shadow:0 0 10px #f44'>--.-<span class='seg-unit'>&deg;C</span></div></div>";
  h += "<div class='seg-box' style='border-color:#f80'><div class='seg-label'>MLX Avg</div><div class='seg-value' id='sv4' style='color:#fa0;text-shadow:0 0 8px #fa0'>--.-<span class='seg-unit'>&deg;C</span></div></div>";
  h += "</div>";

  // === Row 2: Battery tester displays ===
  h += "<div class='seg-display'>";
  h += "<div class='seg-box'><div class='seg-label'>Voltage</div><div class='seg-value' id='sv5' style='color:#0cf;text-shadow:0 0 10px #0cf'>--.---<span class='seg-unit'>V</span></div></div>";
  h += "<div class='seg-box'><div class='seg-label'>Current</div><div class='seg-value' id='sv6' style='color:#f80;text-shadow:0 0 10px #f80'>--.--<span class='seg-unit'>A</span></div></div>";
  h += "<div class='seg-box'><div class='seg-label'>Power</div><div class='seg-value' id='sv7' style='color:#fff;text-shadow:0 0 10px #fff'>--.-<span class='seg-unit'>W</span></div></div>";
  h += "</div>";

  // Info bar
  h += "<div class='info-row' style='margin:0 0 10px;padding:4px 10px'>";
  h += "<span>DS18B20: <span class='v' id='cnt'>-</span> sensors</span>";
  h += "<span>MLX: <span class='v' id='mlxst'>-</span></span>";
  h += "<span>Battery: <span class='v' id='btst'>-</span></span>";
  h += "</div>";

  // === Chart panel ===
  h += "<div class='panel'><h2>Real-time Data</h2>";
  // Time range buttons
  h += "<div style='text-align:center;margin-bottom:8px'>";
  h += "<button class='tbtn' onclick='setRange(300)'>5m</button>";
  h += "<button class='tbtn' onclick='setRange(900)'>15m</button>";
  h += "<button class='tbtn active' onclick='setRange(1800)'>30m</button>";
  h += "<button class='tbtn' onclick='setRange(3600)'>1h</button>";
  h += "<button class='tbtn' onclick='setRange(0)'>All</button>";
  h += "<button class='tbtn' onclick='clearChart()' style='margin-left:10px'>Clear</button>";
  h += "</div>";
  h += "<div class='graph-container'><canvas id='tc'></canvas></div>";
  h += "</div>";

  // === Log Panel (compact) ===
  h += "<div class='panel'><h2>Temperature Log</h2>";
  h += "<div class='row'><span class='k'>Status</span><span class='v' id='logSt'>--</span></div>";
  h += "<div class='row'><span class='k'>File Size</span><span class='v' id='logSz'>--</span></div>";
  h += "<div class='row'><span class='k'>Free Space</span><span class='v' id='logFr'>--</span></div>";
  h += "<div style='padding:6px 0;text-align:center'>";
  h += "<button class='btn bg' onclick='logCmd(\"startlog\")'>Start</button>";
  h += "<button class='btn br' onclick='logCmd(\"stoplog\")'>Stop</button>";
  h += "<button class='btn bb' onclick='location.href=\"/download\"'>CSV</button>";
  h += "<button class='btn by' onclick='if(confirm(\"Delete log?\"))logCmd(\"deletelog\")'>Del</button>";
  h += "</div></div>";

  // ========== JavaScript ==========
  h += "<script>";
  h += "function $(i){return document.getElementById(i)}";

  // --- Data storage (localStorage) ---
  h += "var HKEY='lyrat_hist',MAXPTS=3600,MAXAGE=7200000;";
  h += "var hist=[],range=1800;";
  h += "var btV=null,btI=null,btP=null;";

  // Load history from localStorage
  h += "try{var s=localStorage.getItem(HKEY);if(s){hist=JSON.parse(s);";
  h += "var now=Date.now(),cutoff=now-MAXAGE;";
  h += "hist=hist.filter(function(p){return p[0]>cutoff});}}catch(e){hist=[];}";

  h += "function saveHist(){try{localStorage.setItem(HKEY,JSON.stringify(hist))}catch(e){}}";
  h += "function decimate(){if(hist.length<=MAXPTS)return;var n=[];";
  h += "var step=Math.ceil(hist.length/(MAXPTS/2));";
  h += "for(var i=0;i<hist.length;i+=step)n.push(hist[i]);hist=n;}";

  // --- Chart.js setup (same style as battery tester) ---
  h += "var labels=[],t1D=[],t2D=[],mxD=[],maD=[],vD=[],iD=[],pD=[];";
  h += "var ctx=$('tc').getContext('2d');";
  h += "var chart=new Chart(ctx,{type:'line',data:{labels:labels,datasets:[";
  h += "{label:'T1 (°C)',data:t1D,borderColor:'#0f0',backgroundColor:'rgba(0,255,0,0.05)',yAxisID:'y',tension:0.3,pointRadius:0},";
  h += "{label:'T2 (°C)',data:t2D,borderColor:'#0ff',backgroundColor:'rgba(0,255,255,0.05)',yAxisID:'y',tension:0.3,pointRadius:0},";
  h += "{label:'MLX Max (°C)',data:mxD,borderColor:'#f44',backgroundColor:'rgba(255,68,68,0.05)',yAxisID:'y',tension:0.3,pointRadius:0},";
  h += "{label:'MLX Avg (°C)',data:maD,borderColor:'#fa0',backgroundColor:'rgba(255,170,0,0.05)',yAxisID:'y',tension:0.3,pointRadius:0},";
  h += "{label:'Voltage (V)',data:vD,borderColor:'#0cf',backgroundColor:'rgba(0,204,255,0.1)',yAxisID:'y1',tension:0.3,pointRadius:0},";
  h += "{label:'Current (A)',data:iD,borderColor:'#f80',backgroundColor:'rgba(255,136,0,0.1)',yAxisID:'y1',tension:0.3,pointRadius:0},";
  h += "{label:'Power (W)',data:pD,borderColor:'#fff',backgroundColor:'rgba(255,255,255,0.05)',yAxisID:'y1',tension:0.3,pointRadius:0}";
  h += "]},options:{responsive:true,maintainAspectRatio:false,animation:{duration:0},interaction:{intersect:false,mode:'index'},";
  h += "scales:{x:{display:true,grid:{color:'#333'},ticks:{color:'#888',maxTicksLimit:6}},";
  h += "y:{type:'linear',position:'left',title:{display:true,text:'Temp (°C)',color:'#0f0'},grid:{color:'#333'},ticks:{color:'#0f0'}},";
  h += "y1:{type:'linear',position:'right',title:{display:true,text:'V / A / W',color:'#0cf'},grid:{drawOnChartArea:false},ticks:{color:'#0cf'}}";
  h += "},plugins:{legend:{labels:{color:'#fff',usePointStyle:true}}}}});";

  // Rebuild chart from localStorage history
  h += "function rebuildChart(){";
  h += "labels.length=0;t1D.length=0;t2D.length=0;mxD.length=0;maD.length=0;vD.length=0;iD.length=0;pD.length=0;";
  h += "var now=Date.now(),data;";
  h += "if(range>0){var co=now-range*1000;data=hist.filter(function(p){return p[0]>co});}else{data=hist;}";
  h += "for(var i=0;i<data.length;i++){var d=data[i];";
  h += "labels.push(new Date(d[0]).toLocaleTimeString());";
  h += "t1D.push(d[1]);t2D.push(d[2]);mxD.push(d[3]);maD.push(d[4]);";
  h += "vD.push(d[5]);iD.push(d[6]);pD.push(d[7]);}";
  h += "chart.update();}";

  h += "function clearChart(){hist=[];saveHist();rebuildChart();}";

  // --- Time range ---
  h += "function setRange(s){range=s;";
  h += "var btns=document.querySelectorAll('.tbtn');";
  h += "btns.forEach(function(b){if(b.textContent!='Clear')b.className='tbtn'});";
  h += "event.target.className='tbtn active';rebuildChart();}";

  // --- Battery tester fetch ---
  h += "function fetchBT(){";
  h += "fetch('http://192.168.1.40/status').then(function(r){return r.json()}).then(function(d){";
  h += "btV=d.v;btI=d.i;btP=d.p;";
  h += "if(btV!=null)$('sv5').innerHTML=btV.toFixed(3)+'<span class=\"seg-unit\">V</span>';";
  h += "if(btI!=null)$('sv6').innerHTML=Math.abs(btI).toFixed(2)+'<span class=\"seg-unit\">A</span>';";
  h += "if(btP!=null)$('sv7').innerHTML=Math.abs(btP).toFixed(1)+'<span class=\"seg-unit\">W</span>';";
  h += "$('btst').innerText='Online';$('btst').style.color='#0f0';";
  h += "}).catch(function(){";
  h += "$('btst').innerText='Offline';$('btst').style.color='#f44';";
  h += "});}";

  // --- Main update ---
  h += "function upd(){fetch('/status').then(function(r){return r.json()}).then(function(d){";
  h += "var t1=d.t1>-100?d.t1:null,t2=d.t2>-100?d.t2:null;";
  h += "$('sv1').innerHTML=t1!=null?t1.toFixed(1)+'<span class=\"seg-unit\">°C</span>':'N/C';";
  h += "$('sv2').innerHTML=t2!=null?t2.toFixed(1)+'<span class=\"seg-unit\">°C</span>':'N/C';";
  h += "$('sv3').innerHTML=d.mlxMax.toFixed(1)+'<span class=\"seg-unit\">°C</span>';";
  h += "$('sv4').innerHTML=d.mlxAvg.toFixed(1)+'<span class=\"seg-unit\">°C</span>';";
  h += "$('cnt').innerText=d.dsCount;";
  h += "$('mlxst').innerText=d.mlxOk?'Connected':'NOT FOUND';";
  h += "$('mlxst').style.color=d.mlxOk?'#0f0':'#f44';";
  // Add to history
  h += "var pt=[Date.now(),t1,t2,d.mlxMax,d.mlxAvg,btV,btI,btP];";
  h += "hist.push(pt);";
  h += "var cutoff=Date.now()-MAXAGE;";
  h += "while(hist.length>0&&hist[0][0]<cutoff)hist.shift();";
  h += "decimate();saveHist();";
  // Add to chart (live append)
  h += "var now=new Date().toLocaleTimeString();";
  h += "labels.push(now);t1D.push(t1);t2D.push(t2);mxD.push(d.mlxMax);maD.push(d.mlxAvg);";
  h += "vD.push(btV);iD.push(btI);pD.push(btP);";
  // Trim chart to time range
  h += "if(range>0){var maxPts=Math.ceil(range/2);";
  h += "while(labels.length>maxPts){labels.shift();t1D.shift();t2D.shift();mxD.shift();maD.shift();vD.shift();iD.shift();pD.shift();}}";
  h += "chart.update();";
  h += "}).catch(function(){});}";

  // --- Log status ---
  h += "function updLog(){fetch('/loginfo').then(function(r){return r.json()}).then(function(d){";
  h += "$('logSt').innerText=d.logging?'LOGGING':'Idle';";
  h += "$('logSt').style.color=d.logging?'#0f0':'#888';";
  h += "$('logSz').innerText=(d.size/1024).toFixed(1)+' KB';";
  h += "$('logFr').innerText=(d.freeSpace/1024).toFixed(0)+' KB';";
  h += "}).catch(function(){});}";
  h += "function logCmd(c){fetch('/'+c).then(function(r){return r.json()}).then(function(){updLog()}).catch(function(){});}";

  // --- Init ---
  h += "rebuildChart();";
  h += "setInterval(upd,2000);setInterval(fetchBT,2000);setInterval(updLog,5000);";
  h += "upd();fetchBT();updLog();";
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

  Serial.println("\n========== LyraT Sensor Hub v9.0 ==========\n");

  pinMode(BLUE_LED_PIN, OUTPUT);
  digitalWrite(BLUE_LED_PIN, LOW);

  // SPIFFS
  initSPIFFS();

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

  // ArduinoOTA
  ArduinoOTA.setHostname("lyrat-sensor");
  ArduinoOTA.onStart([]() { Serial.println("[OTA] Start"); });
  ArduinoOTA.onEnd([]() { Serial.println("[OTA] Done"); });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("[OTA] %u%%\r", progress * 100 / total);
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("[OTA] Error %u\n", error);
  });
  ArduinoOTA.begin();
  Serial.println("[OTA] Ready");

  // Web server routes
  server.on("/", handleRoot);
  server.on("/status", handleStatus);
  server.on("/thermaldata", handleThermalData);
  server.on("/rescan", handleRescan);
  server.on("/startlog", handleStartLog);
  server.on("/stoplog", handleStopLog);
  server.on("/download", handleDownload);
  server.on("/deletelog", handleDeleteLog);
  server.on("/loginfo", handleLogInfo);
  server.begin();

  // Initial temp request
  dsRequestTemps();

  digitalWrite(BLUE_LED_PIN, HIGH);
  Serial.printf("\nReady: http://%s\n", WiFi.localIP().toString().c_str());
}

// ============== LOOP ==============
void loop() {
  ArduinoOTA.handle();
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

  // Append temp log entry (every 2s when logging)
  appendTempLog();

  // LED heartbeat (fast blink when logging)
  static unsigned long lastBlink = 0;
  unsigned long blinkInterval = loggingEnabled ? 200 : 1000;
  if (millis() - lastBlink > blinkInterval) {
    lastBlink = millis();
    digitalWrite(BLUE_LED_PIN, !digitalRead(BLUE_LED_PIN));
  }

  delay(2);
  yield();
}
