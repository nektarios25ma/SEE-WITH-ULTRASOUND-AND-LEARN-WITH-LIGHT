// NEKTARIOS KOURAKIS - Ultrasonic + MCP9808 + LCD + WiFi AP + HTML Plot + Fit v(T)
// + (κρατάω και IR parts αν θες αργότερα, αλλά το experiment mode δουλεύει μόνο με Ultrasonic)

#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include "DFRobot_MCP9808.h"

// ---------------- I2C pins ----------------
#define SDA_PIN 21
#define SCL_PIN 22

// ---------------- MCP9808 ----------------
#define I2C_ADDRESS  MCP9808_ADDRESS_7
DFRobot_MCP9808_I2C mcp9808(&Wire, I2C_ADDRESS);

// ---------------- LCD ----------------
int lcdColumns = 16;
int lcdRows = 2;
LiquidCrystal_I2C lcd(0x27, lcdColumns, lcdRows);

// ---------------- Ultrasonic ----------------
const int trigPin = 5;
const int echoPin = 18;

volatile long duration_us = 0;     // ultrasonic TOF in microseconds (latest)
volatile float distanceCm = 0;     // ultrasonic distance in cm (latest)

// ---------------- WiFi AP ----------------
const char* AP_SSID = "ESP32-TOF";
const char* AP_PASS = "12345678";
WebServer server(80);

// ---------------- Timing ----------------
unsigned long lastMeasureMs = 0;
unsigned long lastLcdMs = 0;
unsigned long lastSerialMs = 0;

const unsigned long measurePeriodMs = 200;
const unsigned long lcdPeriodMs = 1000;
const unsigned long serialPeriodMs = 1000;

// ---------------- Experiment (mode 2/3) ----------------
// Fix distance = 40.0 cm = 0.40 m
constexpr float FIX_DIST_M = 0.40f;
constexpr float TWO_DIST_M = 2.0f * FIX_DIST_M; // 0.80 m

volatile int expMode = 0; // 0=normal, 2=collect, 3=fit (fit γίνεται on-demand)

// Buffer samples for v(T)
constexpr int MAX_SAMPLES = 300;
volatile int sampleCount = 0;
float sampT[MAX_SAMPLES];
float sampV[MAX_SAMPLES];

// Fit results v(T) = a + bT
volatile bool fitReady = false;
volatile float fitA = 0.0f;
volatile float fitB = 0.0f;
volatile float fitR2 = 0.0f;

// ---------------- Helpers ----------------
float computeSpeed_mps_from_duration(long dur_us) {
  if (dur_us <= 0) return NAN;
  float t_s = dur_us * 1e-6f;
  if (t_s <= 0.0f) return NAN;
  return TWO_DIST_M / t_s; // v = 2d / t
}

void clearSamples() {
  sampleCount = 0;
  fitReady = false;
  fitA = fitB = fitR2 = 0.0f;
}

void addSample(float T, float v) {
  if (sampleCount >= MAX_SAMPLES) return;
  sampT[sampleCount] = T;
  sampV[sampleCount] = v;
  sampleCount++;
}

// Linear regression v = a + bT
void computeFit() {
  int n = sampleCount;
  if (n < 10) { // θέλουμε “αρκετές” μετρήσεις
    fitReady = false;
    return;
  }

  double Sx=0, Sy=0, Sxx=0, Sxy=0;
  for (int i=0;i<n;i++){
    double x = sampT[i];
    double y = sampV[i];
    Sx += x;
    Sy += y;
    Sxx += x*x;
    Sxy += x*y;
  }

  double denom = (n*Sxx - Sx*Sx);
  if (denom == 0) { fitReady = false; return; }

  double b = (n*Sxy - Sx*Sy) / denom;
  double a = (Sy - b*Sx) / n;

  // R^2
  double ymean = Sy / n;
  double ssTot = 0, ssRes = 0;
  for (int i=0;i<n;i++){
    double y = sampV[i];
    double yhat = a + b*sampT[i];
    ssTot += (y - ymean)*(y - ymean);
    ssRes += (y - yhat)*(y - yhat);
  }
  double r2 = (ssTot > 0) ? (1.0 - ssRes/ssTot) : 0.0;

  fitA = (float)a;
  fitB = (float)b;
  fitR2 = (float)r2;
  fitReady = true;
}

// ---------------- Ultrasonic measure ----------------
void measureUltrasonic() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // timeout 30000us ~ 5m
  long d = pulseIn(echoPin, HIGH, 30000);
  duration_us = d;

  // “κανονική” απόσταση (όχι fix) για ενημέρωση, αν τη θες:
  // distanceCm = duration_us * 0.034342 / 2 (με 20°C) — κρατάμε το παλιό:
  distanceCm = (float)duration_us * 0.034342f / 2.0f;
}

// ---------------- HTML ----------------
String makeHTML() {
  String html = R"rawliteral(
<!DOCTYPE html>
<html lang="el">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>ESP32 v(T)</title>
  <style>
    body { font-family: Arial, sans-serif; margin: 18px; }
    .card { padding: 16px; border: 1px solid #ddd; border-radius: 12px; max-width: 760px; }
    h2 { margin: 0 0 10px 0; }
    button { padding: 10px 14px; margin-right: 10px; border: 1px solid #ccc; border-radius: 10px; background: #fafafa; }
    button:active { transform: scale(0.99); }
    .row { margin: 10px 0; }
    .big { font-size: 1.1rem; margin: 6px 0; }
    .muted { color: #666; margin-top: 10px; }
    canvas { border: 1px solid #eee; border-radius: 12px; width: 100%; max-width: 720px; height: 320px; }
    code { background: #f6f6f6; padding: 2px 6px; border-radius: 6px; }
  </style>
</head>
<body>
  <div class="card">
    <h2>Πείραμα v(T) με Ultrasonic (fix distance = 40.0 cm)</h2>

    <div class="row">
      <button onclick="setMode(2)">2: Start Collect</button>
      <button onclick="doFit()">3: Fit v(T)</button>
      <button onclick="resetSamples()">Reset</button>
    </div>

    <div class="big">Mode: <span id="mode">--</span></div>
    <div class="big">Samples: <span id="n">--</span></div>
    <div class="big">Latest: T = <span id="t">--</span> °C,  v = <span id="v">--</span> m/s</div>

    <div class="big">Fit: <span id="fit">--</span></div>

    <div class="row">
      <canvas id="plot" width="720" height="320"></canvas>
    </div>

    <div class="muted">
      Τύπος: <code>v = 2d / t</code> με d=0.40m και t=duration_us·1e-6 s.
      Ζέσταινε/κρύωνε τον αισθητήρα με πιστολάκι και πάτα <b>3</b> όταν έχεις αρκετά δείγματα.
    </div>
  </div>

<script>
let timer = null;
const cvs = document.getElementById('plot');
const ctx = cvs.getContext('2d');

function clearCanvas(){
  ctx.clearRect(0,0,cvs.width,cvs.height);
}

function drawAxes(){
  ctx.lineWidth = 1;
  ctx.beginPath();
  ctx.moveTo(50, 10);
  ctx.lineTo(50, 300);
  ctx.lineTo(710, 300);
  ctx.stroke();
  ctx.fillText("v (m/s)", 10, 20);
  ctx.fillText("T (°C)", 680, 315);
}

function plotData(points, fit){
  clearCanvas();
  drawAxes();

  if(points.length === 0) return;

  // autoscale
  let Tmin = Math.min(...points.map(p=>p.T));
  let Tmax = Math.max(...points.map(p=>p.T));
  let vmin = Math.min(...points.map(p=>p.v));
  let vmax = Math.max(...points.map(p=>p.v));

  // padding
  if(Tmin === Tmax){ Tmin -= 1; Tmax += 1; }
  if(vmin === vmax){ vmin -= 1; vmax += 1; }
  const Tp = (Tmax-Tmin)*0.08; Tmin -= Tp; Tmax += Tp;
  const vp = (vmax-vmin)*0.08; vmin -= vp; vmax += vp;

  function xpix(T){ return 50 + (T - Tmin) * (660 / (Tmax - Tmin)); }
  function ypix(v){ return 300 - (v - vmin) * (290 / (vmax - vmin)); }

  // points
  ctx.beginPath();
  for(const p of points){
    const x = xpix(p.T);
    const y = ypix(p.v);
    ctx.moveTo(x+2, y);
    ctx.arc(x, y, 2, 0, Math.PI*2);
  }
  ctx.fill();

  // fit line if available
  if(fit && fit.ready){
    const a = fit.a, b = fit.b;
    const T1 = Tmin, T2 = Tmax;
    const v1 = a + b*T1;
    const v2 = a + b*T2;
    ctx.beginPath();
    ctx.moveTo(xpix(T1), ypix(v1));
    ctx.lineTo(xpix(T2), ypix(v2));
    ctx.stroke();
  }
}

async function fetchJSON(url){
  const r = await fetch(url, {cache:'no-store'});
  return await r.json();
}

async function refresh(){
  const j = await fetchJSON('/data');
  document.getElementById('mode').textContent = j.mode;
  document.getElementById('n').textContent = j.n;
  document.getElementById('t').textContent = Number(j.temp_c).toFixed(2);
  document.getElementById('v').textContent = Number(j.v_mps).toFixed(2);

  const fit = j.fit_ready ? `v(T)= ${j.a.toFixed(3)} + ${j.b.toFixed(3)}·T   (R²=${j.r2.toFixed(4)})` : "--";
  document.getElementById('fit').textContent = fit;

  const s = await fetchJSON('/samples');
  plotData(s.points, {ready:j.fit_ready, a:j.a, b:j.b});
}

async function setMode(m){
  await fetch(`/mode?set=${m}`, {cache:'no-store'});
  if(timer) clearInterval(timer);
  timer = setInterval(refresh, 500);
  refresh();
}

async function doFit(){
  await fetch('/fit', {cache:'no-store'});
  if(!timer){
    timer = setInterval(refresh, 500);
  }
  refresh();
}

async function resetSamples(){
  await fetch('/reset', {cache:'no-store'});
  refresh();
}

setMode(0);
</script>
</body>
</html>
)rawliteral";
  return html;
}

// ---------------- Web handlers ----------------
void handleRoot() {
  server.send(200, "text/html; charset=utf-8", makeHTML());
}

// mode set: /mode?set=2 or /mode?set=0
void handleMode() {
  if (server.hasArg("set")) {
    int m = server.arg("set").toInt();
    if (m == 0) { expMode = 0; }
    if (m == 2) { expMode = 2; fitReady = false; } // συλλογή
  }
  server.send(200, "text/plain", "OK");
}

void handleReset() {
  clearSamples();
  expMode = 0;
  server.send(200, "text/plain", "RESET");
}

// Fit endpoint (button 3)
void handleFit() {
  computeFit();
  server.send(200, "text/plain", fitReady ? "FIT_OK" : "FIT_NEED_MORE_SAMPLES");
}

// Data endpoint (latest + fit)
void handleData() {
  float T = mcp9808.getTemperature();
  float v = computeSpeed_mps_from_duration(duration_us);

  String json = "{";
  json += "\"mode\":" + String(expMode) + ",";
  json += "\"n\":" + String(sampleCount) + ",";
  json += "\"duration_us\":" + String(duration_us) + ",";
  json += "\"temp_c\":" + String(T, 4) + ",";
  json += "\"v_mps\":" + String(v, 4) + ",";
  json += "\"fit_ready\":" + String(fitReady ? "true" : "false") + ",";
  json += "\"a\":" + String(fitA, 6) + ",";
  json += "\"b\":" + String(fitB, 6) + ",";
  json += "\"r2\":" + String(fitR2, 6);
  json += "}";
  server.send(200, "application/json", json);
}

// Samples endpoint for plotting
void handleSamples() {
  String json = "{ \"points\": [";
  int n = sampleCount;
  for (int i=0;i<n;i++){
    json += "{";
    json += "\"T\":" + String(sampT[i], 4) + ",";
    json += "\"v\":" + String(sampV[i], 4);
    json += "}";
    if (i < n-1) json += ",";
  }
  json += "] }";
  server.send(200, "application/json", json);
}

// ---------------- Setup/Loop ----------------
void setup() {
  Serial.begin(115200);
  delay(300);

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);

  // MCP9808
  while(!mcp9808.begin()){
    Serial.println("MCP9808 begin failed!");
    delay(1000);
  }
  mcp9808.wakeUpMode();
  mcp9808.setResolution(RESOLUTION_0_125);

  // LCD
  lcd.init();
  lcd.backlight();

  // Ultrasonic pins
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  // WiFi AP
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS);
  IPAddress ip = WiFi.softAPIP();

  Serial.println();
  Serial.print("WiFi AP SSID: "); Serial.println(AP_SSID);
  Serial.print("WiFi AP PASS: "); Serial.println(AP_PASS);
  Serial.print("Open: http://"); Serial.println(ip);

  // Web routes
  server.on("/", handleRoot);
  server.on("/data", handleData);
  server.on("/samples", handleSamples);
  server.on("/mode", handleMode);
  server.on("/fit", handleFit);
  server.on("/reset", handleReset);
  server.begin();

  // LCD show IP briefly
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("WiFi: ESP32-TOF");
  lcd.setCursor(0, 1);
  lcd.print(ip);
  delay(5000);
  lcd.clear();
}

void loop() {
  server.handleClient();

  unsigned long now = millis();

  // measure periodically
  if (now - lastMeasureMs >= measurePeriodMs) {
    lastMeasureMs = now;
    measureUltrasonic();

    // Experiment collect mode
    if (expMode == 2) {
      float T = mcp9808.getTemperature();
      float v = computeSpeed_mps_from_duration(duration_us);
      if (!isnan(v) && v > 50.0f && v < 600.0f) { // βασικό φίλτρο
        addSample(T, v);
      }
    }
  }

  // Serial periodically
  if (now - lastSerialMs >= serialPeriodMs) {
    lastSerialMs = now;
    float T = mcp9808.getTemperature();
    float v = computeSpeed_mps_from_duration(duration_us);

    Serial.print("Mode=");
    Serial.print(expMode);
    Serial.print(" | N=");
    Serial.print(sampleCount);
    Serial.print(" | dur=");
    Serial.print(duration_us);
    Serial.print(" us | T=");
    Serial.print(T, 2);
    Serial.print(" C | v=");
    Serial.print(v, 2);
    Serial.println(" m/s");
  }

  // LCD update
  if (now - lastLcdMs >= lcdPeriodMs) {
    lastLcdMs = now;

    float T = mcp9808.getTemperature();
    float v = computeSpeed_mps_from_duration(duration_us);

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("T=");
    lcd.print(T, 1);
    lcd.print("C M=");
    lcd.print(expMode);

    lcd.setCursor(0, 1);
    lcd.print("v=");
    lcd.print(v, 0);
    lcd.print(" N=");
    lcd.print(sampleCount);
  }
}
