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
  h += "<style>";
  h += "body{background:#111;color:#eee;font-family:'Courier New',monospace;margin:0;padding:10px}";
  h += "h1{color:#0af;text-align:center;font-size:1.3em;margin:8px 0}";
  h += ".panel{background:#1a1a2e;border-radius:8px;padding:12px;margin:8px 0}";
  h += ".panel h2{margin:0 0 8px;color:#0cf;font-size:1em}";
  // 7-segment style displays
  h += ".seg-row{display:flex;flex-wrap:wrap;gap:8px;justify-content:center}";
  h += ".seg-box{background:#0a0a1a;border:1px solid #333;border-radius:8px;padding:10px 14px;min-width:120px;text-align:center;flex:1}";
  h += ".seg-label{font-size:0.7em;color:#888;text-transform:uppercase;letter-spacing:1px}";
  h += ".seg-value{font-size:2.2em;font-weight:bold;letter-spacing:2px;margin:4px 0}";
  h += ".seg-unit{font-size:0.6em;color:#888;margin-left:2px}";
  // Info row
  h += ".info-row{display:flex;justify-content:space-between;padding:4px 0;font-size:0.8em;color:#888}";
  h += ".info-row .v{color:#0f0;font-weight:bold}";
  // Chart
  h += "#tc{width:100%;height:250px;border:1px solid #333;border-radius:4px;background:#0a0a1a}";
  h += ".tbtn{padding:5px 12px;border:1px solid #555;border-radius:4px;cursor:pointer;font-family:'Courier New',monospace;font-size:0.8em;margin:2px;background:#222;color:#aaa}";
  h += ".tbtn.active{background:#07f;color:#fff;border-color:#09f}";
  // Legend
  h += ".legend{display:flex;flex-wrap:wrap;gap:8px;font-size:0.75em;padding:6px 0}";
  h += ".legend span{cursor:pointer;padding:2px 6px;border-radius:3px;border:1px solid transparent}";
  h += ".legend span.off{opacity:0.3;text-decoration:line-through}";
  // Log panel
  h += ".row{display:flex;justify-content:space-between;padding:4px 0;border-bottom:1px solid #222;font-size:0.85em}";
  h += ".row .k{color:#888}.row .v{color:#0f0;font-weight:bold}";
  h += ".btn{padding:6px 12px;border:none;border-radius:4px;cursor:pointer;font-family:monospace;font-size:0.8em;margin:2px}";
  h += ".bg{background:#0a0;color:#fff}.br{background:#a00;color:#fff}.bb{background:#07f;color:#fff}.by{background:#a80;color:#fff}";
  h += "</style></head><body>";
  h += "<h1>LyraT Sensor Hub v9.0</h1>";

  // === Seg-box temperature displays ===
  h += "<div class='panel'><h2>Temperature</h2>";
  h += "<div class='seg-row'>";
  h += "<div class='seg-box'><div class='seg-label'>T1 (DS18B20)</div><div class='seg-value' id='sv1' style='color:#0f0'>--</div></div>";
  h += "<div class='seg-box'><div class='seg-label'>T2 (DS18B20)</div><div class='seg-value' id='sv2' style='color:#0ff'>--</div></div>";
  h += "<div class='seg-box'><div class='seg-label'>MLX Max</div><div class='seg-value' id='sv3' style='color:#f44'>--</div></div>";
  h += "<div class='seg-box'><div class='seg-label'>MLX Avg</div><div class='seg-value' id='sv4' style='color:#f80'>--</div></div>";
  h += "</div>";
  h += "<div class='info-row' style='margin-top:6px'><span>DS18B20: <span class='v' id='cnt'>-</span> sensors</span>";
  h += "<span>MLX: <span class='v' id='mlxst'>-</span></span></div>";
  h += "</div>";

  // === Seg-box battery tester displays ===
  h += "<div class='panel'><h2>Battery Tester (192.168.1.40)</h2>";
  h += "<div class='seg-row'>";
  h += "<div class='seg-box'><div class='seg-label'>Voltage</div><div class='seg-value' id='sv5' style='color:#0ff'>--<span class='seg-unit'>V</span></div></div>";
  h += "<div class='seg-box'><div class='seg-label'>Current</div><div class='seg-value' id='sv6' style='color:#f80'>--<span class='seg-unit'>A</span></div></div>";
  h += "<div class='seg-box'><div class='seg-label'>Power</div><div class='seg-value' id='sv7' style='color:#fff'>--<span class='seg-unit'>W</span></div></div>";
  h += "</div>";
  h += "<div class='info-row' style='margin-top:6px'><span>Status: <span class='v' id='btst'>-</span></span></div>";
  h += "</div>";

  // === Chart panel ===
  h += "<div class='panel'><h2>Real-time Chart</h2>";
  // Time range buttons
  h += "<div style='text-align:center;margin-bottom:6px'>";
  h += "<button class='tbtn' onclick='setRange(300)'>5m</button>";
  h += "<button class='tbtn' onclick='setRange(900)'>15m</button>";
  h += "<button class='tbtn active' onclick='setRange(1800)'>30m</button>";
  h += "<button class='tbtn' onclick='setRange(3600)'>1h</button>";
  h += "<button class='tbtn' onclick='setRange(0)'>All</button>";
  h += "</div>";
  h += "<canvas id='tc'></canvas>";
  // Legend
  h += "<div class='legend' id='leg'>";
  h += "<span style='color:#0f0' onclick='togS(0)'>&#9632; T1</span>";
  h += "<span style='color:#0ff' onclick='togS(1)'>&#9632; T2</span>";
  h += "<span style='color:#f44' onclick='togS(2)'>&#9632; MLX Max</span>";
  h += "<span style='color:#f80' onclick='togS(3)'>&#9632; MLX Avg</span>";
  h += "<span style='color:#0af' onclick='togS(4)'>&#9632; Voltage</span>";
  h += "<span style='color:#fa0' onclick='togS(5)'>&#9632; Current</span>";
  h += "<span style='color:#fff' onclick='togS(6)'>&#9632; Power</span>";
  h += "</div></div>";

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
  h += "var $=function(id){return document.getElementById(id)};";

  // --- Data storage ---
  h += "var HKEY='lyrat_hist',MAXPTS=3600,MAXAGE=7200000;"; // max 3600 points, 2h
  h += "var hist=[],range=1800,seriesOn=[1,1,1,1,1,1,1];"; // default 30m
  h += "var btV=null,btI=null,btP=null;"; // battery tester values

  // Load history from localStorage
  h += "try{var s=localStorage.getItem(HKEY);if(s){hist=JSON.parse(s);";
  h += "var now=Date.now(),cutoff=now-MAXAGE;";
  h += "hist=hist.filter(function(p){return p[0]>cutoff});}}catch(e){hist=[];}";

  // Save history to localStorage
  h += "function saveHist(){try{localStorage.setItem(HKEY,JSON.stringify(hist))}catch(e){}}";

  // Decimate if too many points
  h += "function decimate(){if(hist.length<=MAXPTS)return;var n=[];";
  h += "var step=Math.ceil(hist.length/(MAXPTS/2));";
  h += "for(var i=0;i<hist.length;i+=step)n.push(hist[i]);";
  h += "hist=n;}";

  // --- Time range ---
  h += "function setRange(s){range=s;";
  h += "var btns=document.querySelectorAll('.tbtn');";
  h += "btns.forEach(function(b){b.className='tbtn'});";
  h += "event.target.className='tbtn active';drawChart();}";

  // --- Toggle series ---
  h += "function togS(i){seriesOn[i]=seriesOn[i]?0:1;";
  h += "var spans=$('leg').children;";
  h += "spans[i].className=seriesOn[i]?'':'off';drawChart();}";

  // --- Chart drawing ---
  h += "function drawChart(){";
  h += "var cv=$('tc'),ctx=cv.getContext('2d');";
  h += "var dpr=window.devicePixelRatio||1;";
  h += "var w=cv.clientWidth,h=cv.clientHeight;";
  h += "cv.width=w*dpr;cv.height=h*dpr;ctx.scale(dpr,dpr);";
  // Margins
  h += "var ml=50,mr=55,mt=10,mb=30;";
  h += "var pw=w-ml-mr,ph=h-mt-mb;";
  h += "if(pw<10||ph<10)return;";
  // Filter data by time range
  h += "var now=Date.now(),data;";
  h += "if(range>0){var cutoff=now-range*1000;";
  h += "data=hist.filter(function(p){return p[0]>cutoff});}";
  h += "else{data=hist;}";
  h += "if(data.length<2){ctx.fillStyle='#555';ctx.font='14px Courier New';";
  h += "ctx.fillText('Waiting for data...',w/2-70,h/2);return;}";
  // Compute min/max for left Y (temps idx 1-4) and right Y (V/I/P idx 5-7)
  h += "var tMin=999,tMax=-999,eMin=999,eMax=-999;";
  h += "for(var i=0;i<data.length;i++){var d=data[i];";
  h += "for(var j=1;j<=4;j++){if(seriesOn[j-1]&&d[j]!=null){if(d[j]<tMin)tMin=d[j];if(d[j]>tMax)tMax=d[j];}}";
  h += "for(var j=5;j<=7;j++){if(seriesOn[j-1]&&d[j]!=null){if(d[j]<eMin)eMin=d[j];if(d[j]>eMax)eMax=d[j];}}}";
  // Pad ranges
  h += "if(tMin>=tMax){tMin-=1;tMax+=1;}var tPad=(tMax-tMin)*0.1;tMin-=tPad;tMax+=tPad;";
  h += "if(eMin>=eMax){eMin-=0.5;eMax+=0.5;}var ePad=(eMax-eMin)*0.1;eMin-=ePad;eMax+=ePad;";
  h += "var tS=data[0][0],tE=data[data.length-1][0],tR=tE-tS||1;";
  // Background
  h += "ctx.fillStyle='#0a0a1a';ctx.fillRect(0,0,w,h);";
  // Grid
  h += "ctx.strokeStyle='#222';ctx.lineWidth=0.5;";
  h += "for(var i=0;i<=5;i++){var y=mt+ph*(i/5);ctx.beginPath();ctx.moveTo(ml,y);ctx.lineTo(ml+pw,y);ctx.stroke();}";
  // Left Y labels (temps)
  h += "ctx.fillStyle='#0f0';ctx.font='10px Courier New';ctx.textAlign='right';";
  h += "for(var i=0;i<=5;i++){var v=tMax-(tMax-tMin)*(i/5);";
  h += "ctx.fillText(v.toFixed(1),ml-4,mt+ph*(i/5)+4);}";
  // Right Y labels (electrical)
  h += "ctx.fillStyle='#0af';ctx.textAlign='left';";
  h += "for(var i=0;i<=5;i++){var v=eMax-(eMax-eMin)*(i/5);";
  h += "ctx.fillText(v.toFixed(1),ml+pw+4,mt+ph*(i/5)+4);}";
  // X axis labels
  h += "ctx.fillStyle='#888';ctx.textAlign='center';";
  h += "for(var i=0;i<=4;i++){var t=tS+tR*(i/4);";
  h += "var ago=Math.round((now-t)/1000);var mm=Math.floor(ago/60);var ss=ago%60;";
  h += "ctx.fillText('-'+mm+'m'+('0'+ss).slice(-2)+'s',ml+pw*(i/4),h-mb+15);}";
  // Draw series
  h += "var colors=['#0f0','#0ff','#f44','#f80','#0af','#fa0','#fff'];";
  h += "ctx.lineWidth=1.5;";
  h += "for(var s=0;s<7;s++){if(!seriesOn[s])continue;";
  h += "var idx=s+1;var isE=s>=4;";
  h += "var yMin=isE?eMin:tMin,yMax=isE?eMax:tMax,yR=yMax-yMin||1;";
  h += "ctx.strokeStyle=colors[s];ctx.beginPath();var started=0;";
  h += "for(var i=0;i<data.length;i++){var d=data[i];";
  h += "if(d[idx]==null)continue;";
  h += "var x=ml+pw*((d[0]-tS)/tR);";
  h += "var y=mt+ph*(1-(d[idx]-yMin)/yR);";
  h += "if(!started){ctx.moveTo(x,y);started=1;}else{ctx.lineTo(x,y);}}";
  h += "ctx.stroke();}";
  h += "}"; // end drawChart

  // --- Battery tester fetch ---
  h += "function fetchBT(){";
  h += "fetch('http://192.168.1.40/status').then(function(r){return r.json()}).then(function(d){";
  h += "btV=d.v;btI=d.i;btP=d.p;";
  // Update seg-boxes
  h += "if(btV!=null)$('sv5').innerHTML=btV.toFixed(2)+'<span class=\"seg-unit\">V</span>';";
  h += "if(btI!=null)$('sv6').innerHTML=Math.abs(btI).toFixed(2)+'<span class=\"seg-unit\">A</span>';";
  h += "if(btP!=null)$('sv7').innerHTML=Math.abs(btP).toFixed(1)+'<span class=\"seg-unit\">W</span>';";
  h += "$('btst').innerText='Online';$('btst').style.color='#0f0';";
  h += "}).catch(function(){";
  h += "$('btst').innerText='Offline';$('btst').style.color='#f44';";
  h += "});}";

  // --- Main update ---
  h += "function upd(){fetch('/status').then(function(r){return r.json()}).then(function(d){";
  // Update seg-box displays
  h += "var t1=d.t1>-100?d.t1:null,t2=d.t2>-100?d.t2:null;";
  h += "$('sv1').innerHTML=t1!=null?t1.toFixed(1)+'<span class=\"seg-unit\">&deg;C</span>':'N/C';";
  h += "$('sv2').innerHTML=t2!=null?t2.toFixed(1)+'<span class=\"seg-unit\">&deg;C</span>':'N/C';";
  h += "$('sv3').innerHTML=d.mlxMax.toFixed(1)+'<span class=\"seg-unit\">&deg;C</span>';";
  h += "$('sv4').innerHTML=d.mlxAvg.toFixed(1)+'<span class=\"seg-unit\">&deg;C</span>';";
  h += "$('cnt').innerText=d.dsCount;";
  h += "$('mlxst').innerText=d.mlxOk?'Connected':'NOT FOUND';";
  h += "$('mlxst').style.color=d.mlxOk?'#0f0':'#f44';";
  // Add data point: [ts, t1, t2, mlxMax, mlxAvg, voltage, current, power]
  h += "var pt=[Date.now(),t1,t2,d.mlxMax,d.mlxAvg,btV,btI,btP];";
  h += "hist.push(pt);";
  // Trim old data
  h += "var cutoff=Date.now()-MAXAGE;";
  h += "while(hist.length>0&&hist[0][0]<cutoff)hist.shift();";
  h += "decimate();saveHist();drawChart();";
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
  h += "setInterval(upd,2000);setInterval(fetchBT,2000);setInterval(updLog,5000);";
  h += "upd();fetchBT();updLog();";
  h += "window.addEventListener('resize',drawChart);";
  h += "setTimeout(drawChart,100);";
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
