#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include "DFRobot_TMF8x01.h"

WebServer server(80);

// ---------- UART2 pins ----------
static const int RX2_PIN = 16;  // ESP32 receives from UNO TX (through divider)
static const int TX2_PIN = 17;  // ESP32 sends to UNO RX (optional)

// ---------- I2C for TMF8801 ----------
#define SDA_PIN 21
#define SCL_PIN 22

// ---------- Latest from UNO ----------
float tempC = NAN;
long us_tof_us = 0;

// ---------- TMF8801 ----------
constexpr double C_LIGHT = 299792458.0;
DFRobot_TMF8801 tofIR(-1, -1);

uint8_t tmf8801Cal_A[14] = {
  0x33, 0x37, 0x00, 0x41, 0xC5, 0x7E, 0xC9, 0x04, 0x06, 0xEF, 0x57, 0xA4, 0x00, 0xFC
};

uint16_t ir_mm = 0;
float ir_cm = 0;
double ir_tof_ns = NAN;
double ir_avg_ns = 0;

// ---------- WiFi AP ----------
const char* AP_SSID = "ESP32-TOF";
const char* AP_PASS = "12345678";

// ---------- Experiment v(T) ----------
constexpr float FIX_DIST_M = 0.415f;
constexpr float TWO_DIST_M = 0.830f;
static inline float round1(float x){ return roundf(x*10.0f)/10.0f; }

int expMode = 0;
constexpr int MAX_POINTS = 500;
int pointCount = 0;
float ptT[MAX_POINTS];
float ptV[MAX_POINTS];

bool fitReady=false;
float fitA=0, fitB=0, fitR2=0;

constexpr float DV_LIMIT = 2.0f;      // μέγιστη επιτρεπτή μεταβολή v ανά νέο bin
constexpr float DV_SIGN_TOL = 0.8f;   // μικρή ανοχή στο πρόσημο για να μη χάνονται μετρήσεις
constexpr int BASELINE_MIN = 50;
constexpr int STEP_MIN = 4;

constexpr float US_T0_US = 27.0f;   // σταθερή καθυστέρηση ultrasonic (μs)

bool haveBaseline=false;
int lastAcceptedT=0;
float lastAcceptedV=0;

bool haveCurrentBin=false;
int curBinT=0;
int accN=0;
float accSumV=0;

float speedFromUltrasonic(long dur_us){
  float dur_corr = (float)dur_us - US_T0_US;
  if(dur_corr <= 1.0f) return NAN;
  float t = dur_corr * 1e-6f;
  return TWO_DIST_M / t;
}

void clearExperiment(){
  expMode=0;
  pointCount=0;
  fitReady=false; fitA=fitB=fitR2=0;
  haveBaseline=false; lastAcceptedT=0; lastAcceptedV=0;
  haveCurrentBin=false; curBinT=0; accN=0; accSumV=0;
}

void addPoint(int Tbin, float v1){
  if(pointCount>=MAX_POINTS) return;
  ptT[pointCount]=(float)Tbin;
  ptV[pointCount]=v1;
  pointCount++;
}

void resetBin(int newBinT){
  curBinT=newBinT;
  accN=0;
  accSumV=0;
  haveCurrentBin=true;
}

void finalizeCurrentBin(){
  if(!haveCurrentBin) return;

  int required = (pointCount==0) ? BASELINE_MIN : STEP_MIN;
  if(accN < required) return;

  float vmean1 = round1(accSumV / (float)accN);

  if(pointCount==0){
    addPoint(curBinT, vmean1);
    haveBaseline=true;
    lastAcceptedT=curBinT;
    lastAcceptedV=vmean1;
    return;
  }

  int dTbin = curBinT - lastAcceptedT;
  if(dTbin == 0) return;  // 1 σημείο ανά ακέραιο °C bin

  float dV = vmean1 - lastAcceptedV;
  bool ok = false;

  // Physics-based acceptance filter με μικρή ανοχή στο πρόσημο
  if (dTbin > 0) {
    ok = (dV >= -DV_SIGN_TOL && dV < DV_LIMIT);
  } else if (dTbin < 0) {
    ok = (dV <=  DV_SIGN_TOL && dV > -DV_LIMIT);
  }

  if(ok){
    addPoint(curBinT, vmean1);
    lastAcceptedT=curBinT;
    lastAcceptedV=vmean1;
  }
}

void updateCollect(){
  if(expMode!=2) return;
  if(isnan(tempC)) return;

  float vraw = speedFromUltrasonic(us_tof_us);
  if(isnan(vraw) || vraw<50 || vraw>600) return;

  int Tbin = (int)lroundf(tempC);

  if(!haveCurrentBin){
    resetBin(Tbin);
  }

  // Όταν αλλάξει το rounded θερμοκρασιακό bin, κλείνουμε το προηγούμενο
  if(Tbin != curBinT){
    finalizeCurrentBin();
    resetBin(Tbin);
  }

  accSumV += vraw;
  accN++;
}

void flushCurrentBinForFit(){
  finalizeCurrentBin();
}

void computeFit(){
  int n=pointCount;
  if(n<10){ fitReady=false; return; }

  double Sx=0,Sy=0,Sxx=0,Sxy=0;
  for(int i=0;i<n;i++){
    double x=ptT[i], y=ptV[i];
    Sx+=x; Sy+=y; Sxx+=x*x; Sxy+=x*y;
  }
  double denom = n*Sxx - Sx*Sx;
  if(denom==0){ fitReady=false; return; }

  double b = (n*Sxy - Sx*Sy)/denom;
  double a = (Sy - b*Sx)/n;

  double ymean = Sy/n, ssTot=0, ssRes=0;
  for(int i=0;i<n;i++){
    double y=ptV[i], yhat=a+b*ptT[i];
    ssTot += (y-ymean)*(y-ymean);
    ssRes += (y-yhat)*(y-yhat);
  }
  double r2 = (ssTot>0) ? (1.0 - ssRes/ssTot) : 0;

  fitA=a; fitB=b; fitR2=r2; fitReady=true;
}

// ---------- TMF8801 ----------
bool readIROnce(uint16_t &d_mm, uint32_t timeout_ms=3000){
  uint32_t t0=millis();
  while(!tofIR.isDataReady()){
    if(millis()-t0 > timeout_ms) return false;
    delay(5);
  }
  d_mm = tofIR.getDistance_mm();
  return true;
}

void initTMF8801(){
  while(tofIR.begin()!=0) delay(400);

  tofIR.stopMeasurement(); delay(50);
  tofIR.setCalibrationData(tmf8801Cal_A, 14);
  tofIR.startMeasurement(tofIR.eModeCalib);
  delay(150);

  uint16_t dmm;
  if(!readIROnce(dmm, 3000)){
    tofIR.stopMeasurement(); delay(50);
    tofIR.startMeasurement(tofIR.eModeNoCalib);
    delay(150);
  }
}

void updateIR(){
  if(!tofIR.isDataReady()) return;

  uint16_t dmm = tofIR.getDistance_mm();

  double d_m = dmm / 1000.0;
  double t_ns = ((2.0 * d_m) / C_LIGHT) * 1e9;

  if (t_ns < 2.600 || t_ns > 2.730) {
    return;
  }

  ir_mm = dmm;
  ir_cm = dmm / 10.0f;
  ir_tof_ns = t_ns;

  static double avg = 0; static uint32_t n = 0;
  avg += (t_ns - avg) / (double)(++n > 100 ? (n = 100) : n);
  ir_avg_ns = avg;
}


void sendIRToUNO(){
  static unsigned long lastSendMs = 0;
  if(millis() - lastSendMs < 200) return;   // περίπου 5 Hz προς LCD
  lastSendMs = millis();

  double out = ir_avg_ns;
  if((isnan(out) || out <= 0.0) && !isnan(ir_tof_ns) && ir_tof_ns > 0.0) {
    out = ir_tof_ns;
  }
  if(isnan(out) || out <= 0.0) return;

  // Γραμμή προς UNO: IR,2.7
  Serial2.print("IR,");
  Serial2.println(out, 1);
}

// ---------- Web ----------
String makeHTML(){
  String html = R"rawliteral(
<!doctype html><html lang="el"><head>
<meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>ESP32 TOF + v(T)</title>
<style>
body{font-family:Arial;margin:18px}
.card{padding:16px;border:1px solid #ddd;border-radius:12px;max-width:860px}
.big{margin:6px 0;font-size:1.05rem}
button{padding:10px 14px;margin-right:10px;border:1px solid #ccc;border-radius:10px;background:#fafafa}
canvas{border:1px solid #eee;border-radius:12px;width:100%;max-width:820px;height:340px}
hr{border:0;border-top:1px solid #eee;margin:12px 0}
</style></head><body>
<div class="card">
<h2>UNO (Ultrasonic+Temp) + ESP32 (TMF8801 IR)</h2>

<div class="big"><b>Ultrasonic TOF:</b> <span id="us">--</span> μs</div>
<div class="big"><b>IR ToF:</b> <span id="irtof">--</span> ns (avg <span id="iravg">--</span>)</div>
<div class="big"><b>IR Distance:</b> <span id="irmm">--</span> mm (<span id="ircm">--</span> cm)</div>
<div class="big"><b>Temp:</b> <span id="t">--</span> °C | <b>v:</b> <span id="v">--</span> m/s</div>

<hr>
<button onclick="setMode(2)">2: Start Collect</button>
<button onclick="doFit()">3: Fit v(T)</button>
<button onclick="resetAll()">Reset</button>

<div class="big">Mode: <span id="mode">--</span> | Points: <span id="npt">--</span> | Bin samples: <span id="bn">--</span></div>
<div class="big">Fit: <span id="fit">--</span></div>

<canvas id="plot" width="820" height="340"></canvas>
</div>

<script>
const cvs=document.getElementById('plot'); const ctx=cvs.getContext('2d');
function clearC(){ctx.clearRect(0,0,cvs.width,cvs.height)}
function axes(){ctx.beginPath();ctx.moveTo(50,10);ctx.lineTo(50,320);ctx.lineTo(810,320);ctx.stroke();ctx.fillText("v (m/s)",10,20);ctx.fillText("T (°C)",785,335)}
function plot(points,fit){
  clearC(); axes(); if(!points.length) return;
  let Tmin=Math.min(...points.map(p=>p.T)), Tmax=Math.max(...points.map(p=>p.T));
  let vmin=Math.min(...points.map(p=>p.v)), vmax=Math.max(...points.map(p=>p.v));
  if(Tmin===Tmax){Tmin-=1;Tmax+=1} if(vmin===vmax){vmin-=1;vmax+=1}
  const Tp=(Tmax-Tmin)*0.08; Tmin-=Tp; Tmax+=Tp; const vp=(vmax-vmin)*0.08; vmin-=vp; vmax+=vp;
  const xpix=T=>50+(T-Tmin)*(760/(Tmax-Tmin));
  const ypix=v=>320-(v-vmin)*(300/(vmax-vmin));
  ctx.beginPath();
  for(const p of points){const x=xpix(p.T), y=ypix(p.v); ctx.moveTo(x+2,y); ctx.arc(x,y,2,0,Math.PI*2);}
  ctx.fill();
  if(fit && fit.ready){
    const a=fit.a,b=fit.b;
    const T1=Tmin,T2=Tmax;
    ctx.beginPath(); ctx.moveTo(xpix(T1), ypix(a+b*T1)); ctx.lineTo(xpix(T2), ypix(a+b*T2)); ctx.stroke();
  }
}
async function j(url){return (await fetch(url,{cache:'no-store'})).json();}
async function refresh(){
  const d=await j('/data');
  document.getElementById('us').textContent=d.us_tof_us;
  document.getElementById('irtof').textContent=Number(d.ir_tof_ns).toFixed(3);
  document.getElementById('iravg').textContent=Number(d.ir_avg_ns).toFixed(3);
  document.getElementById('irmm').textContent=d.ir_mm;
  document.getElementById('ircm').textContent=Number(d.ir_cm).toFixed(2);
  document.getElementById('t').textContent=Number(d.temp_c).toFixed(2);
  document.getElementById('v').textContent=Number(d.v_mps).toFixed(1);
  document.getElementById('mode').textContent=d.mode;
  document.getElementById('npt').textContent=d.points;
  document.getElementById('bn').textContent=d.bin_n;
  document.getElementById('fit').textContent = d.fit_ready ? `v(T)= ${d.a.toFixed(3)} + ${d.b.toFixed(3)}·T (R²=${d.r2.toFixed(4)})` : "--";
  const s=await j('/samples');
  plot(s.points,{ready:d.fit_ready,a:d.a,b:d.b});
}
async function setMode(m){ await fetch(`/mode?set=${m}`,{cache:'no-store'}); }
async function doFit(){ await fetch('/fit',{cache:'no-store'}); }
async function resetAll(){ await fetch('/reset',{cache:'no-store'}); }
setInterval(refresh,500); refresh();
</script>
</body></html>
)rawliteral";
  return html;
}

void handleRoot(){ server.send(200,"text/html; charset=utf-8",makeHTML()); }
void handleMode(){ if(server.hasArg("set")){ int m=server.arg("set").toInt(); if(m==0) expMode=0; if(m==2){ clearExperiment(); expMode=2; } } server.send(200,"text/plain","OK"); }
void handleReset(){ clearExperiment(); server.send(200,"text/plain","RESET"); }
void handleFit(){ flushCurrentBinForFit(); computeFit(); server.send(200,"text/plain", fitReady ? "FIT_OK":"FIT_NEED_MORE_POINTS"); }

void handleData(){
  float v = speedFromUltrasonic(us_tof_us);
  float v1 = isnan(v) ? NAN : round1(v);

  String json="{";
  json += "\"temp_c\":"+String(tempC,4)+",";
  json += "\"us_tof_us\":"+String(us_tof_us)+",";
  json += "\"v_mps\":"+String(v1,4)+",";
  json += "\"ir_mm\":"+String(ir_mm)+",";
  json += "\"ir_cm\":"+String(ir_cm,4)+",";
  json += "\"ir_tof_ns\":"+String(ir_tof_ns,6)+",";
  json += "\"ir_avg_ns\":"+String(ir_avg_ns,6)+",";

  json += "\"mode\":"+String(expMode)+",";
  json += "\"points\":"+String(pointCount)+",";
  json += "\"bin_n\":"+String(accN)+",";

  json += "\"fit_ready\":"+(fitReady?String("true"):String("false"))+",";
  json += "\"a\":"+String(fitA,6)+",";
  json += "\"b\":"+String(fitB,6)+",";
  json += "\"r2\":"+String(fitR2,6);
  json += "}";
  server.send(200,"application/json",json);
}

void handleSamples(){
  String json="{\"points\":[";
  for(int i=0;i<pointCount;i++){
    json += "{\"T\":"+String(ptT[i],1)+",\"v\":"+String(ptV[i],1)+"}";
    if(i<pointCount-1) json+=",";
  }
  json += "]}";
  server.send(200,"application/json",json);
}

// UART line from UNO: "T,US_us\n"
String lineBuf;

void parseUNO(const String& s){
  int p=s.indexOf(',');
  if(p<0) return;
  tempC = s.substring(0,p).toFloat();
  us_tof_us = s.substring(p+1).toInt();
}

void setup(){
  Serial.begin(115200);

  Serial2.begin(115200, SERIAL_8N1, RX2_PIN, TX2_PIN);

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);

  initTMF8801();

  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS);

  server.on("/", handleRoot);
  server.on("/data", handleData);
  server.on("/samples", handleSamples);
  server.on("/mode", handleMode);
  server.on("/fit", handleFit);
  server.on("/reset", handleReset);
  server.begin();

  clearExperiment();
}

void loop(){
  server.handleClient();

  updateIR();
  sendIRToUNO();

  while(Serial2.available()){
    char c=(char)Serial2.read();
    if(c=='\n'){
      parseUNO(lineBuf);
      lineBuf="";
      updateCollect();
    } else if(c!='\r'){
      if(lineBuf.length()<80) lineBuf += c;
    }
  }
}
