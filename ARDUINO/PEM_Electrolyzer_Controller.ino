/* ============================================================
   ARDUINO GIGA R1 - PEM ELECTROLYZER CONTROLLER
   INDUSTRIAL GRADE WITH MQTT 3-WAY COMMUNICATION
   Features: Nextion HMI, GSM, Dual Power Control, MPC Integration
   ============================================================ */

#include <Wire.h>
#include "RTClib.h"
#include <WiFi.h>
#include <ArduinoJson.h>
#include <math.h>
#include <PubSubClient.h>

// ========== HARDWARE DEFINITIONS ==========
#define BAT_PIN A0
#define TEMP_SENSOR A1
#define VOLTAGE_SENSOR A2
#define CURRENT_SENSOR A3
#define GRID_RELAY 8
#define PV_RELAY 9

// ========== NETWORK CONFIGURATION ==========
const char* ssid = "Galaxy A14 FD8D";
const char* password = "qwertyzxcvbnm";

// MQTT Configuration
const char* mqttBroker = "broker.hivemq.com";
const int mqttPort = 1883;
const char* mqttClientId = "arduino_pem_giga_industrial";

// ========== HARDWARE SERIAL PORTS ==========
HardwareSerial &nexSerial = Serial3;  // Nextion HMI
HardwareSerial &simSerial = Serial2;  // GSM Module
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);
RTC_DS3231 rtc;

// ========== TIMING CONFIGURATION ==========
unsigned long lastFullUpdate = 0;
unsigned long lastMqttUpdate = 0;
unsigned long lastSafetyCheck = 0;
unsigned long lastMPCUpdate = 0;

const unsigned long FULL_UPDATE_MS = 5000;
const unsigned long MQTT_UPDATE_MS = 1000;
const unsigned long SAFETY_CHECK_MS = 500;
const unsigned long MPC_UPDATE_MS = 100;

// ========== STATE MACHINE ==========
enum SystemMode { MODE_NONE, MODE_AUTO, MODE_MANUAL, MODE_MPC, MODE_ECONOMIC };
enum SystemState { STATE_WAITING, STATE_STARTING, STATE_RUNNING, STATE_STOPPING, STATE_STOPPED };

SystemMode systemMode = MODE_NONE;
SystemState systemState = STATE_WAITING;
unsigned long stateStartTime = 0;
const unsigned long START_DELAY = 2000;
const unsigned long STOP_DELAY = 2000;

// ========== CONNECTIVITY STATUS ==========
bool wifiOK = false;
bool gsmOK = false;
bool mqttConnected = false;
int gsmRssiRaw = 99;

// ========== CONTROL VARIABLES ==========
float prodRateSetpoint = 30.0f;
float mpcOptimalCurrent = 150.0f;
float appliedCurrent = 150.0f;
float manualCurrentSetpoint = 150.0f;
bool sliderMoved = false;
bool mpcControlActive = false;

// ========== DUAL POWER CONTROL ==========
struct PowerConfig {
  String source = "auto";
  float gridRatio = 0.5;
  float pvRatio = 0.5;
  bool gridAvailable = true;
  bool pvAvailable = true;
  float gridPrice = 0.15;
  float pvPrice = 0.05;
} powerConfig;

// ========== MPC PARAMETERS ==========
const int MPC_HORIZON = 5;
const float MPC_SAMPLE_TIME = 0.1;
float mpcStates[MPC_HORIZON] = {0};
float mpcControls[MPC_HORIZON] = {0};
float mpcReference[MPC_HORIZON] = {0};

// System model parameters
const float A_MATRIX = 0.95;
const float B_MATRIX = 0.8;
const float C_MATRIX = 1.0;

// MPC weights
const float Q_WEIGHT = 10.0;
const float R_WEIGHT = 1.0;
const float S_WEIGHT = 100.0;

// ========== SAFETY PARAMETERS ==========
const float MAX_CURRENT = 200.0f;
const float MIN_CURRENT = 100.0f;
const float MAX_TEMPERATURE = 80.0f;
const float MIN_O2_PURITY = 99.0f;
const float MAX_VOLTAGE = 45.0f;

// ========== SENSOR VALUES ==========
float cellTemperature = 65.0f;
float stackVoltage = 38.0f;
float stackCurrent = 150.0f;
float o2Purity = 99.5f;
float h2ProductionRate = 0.0f;
float o2ProductionRate = 0.0f;

// ========== ADC CONFIGURATION ==========
const int ADC_BITS = 12;
const float ADC_REF_VOLTAGE = 3.3f;
const float R_TOP = 1000000.0f;
const float R_BOTTOM = 100000.0f;
const float VOLTAGE_DIVIDER_FACTOR = (R_TOP + R_BOTTOM) / R_BOTTOM;

// ========== MPC CONTROLLER CLASS ==========
class IndustrialMPC {
private:
    float states[MPC_HORIZON];
    float controls[MPC_HORIZON];
    float reference[MPC_HORIZON];
    
public:
    IndustrialMPC() {
        for(int i = 0; i < MPC_HORIZON; i++) {
            states[i] = 0.0f;
            controls[i] = 0.0f;
            reference[i] = 0.0f;
        }
    }
    
    void setReference(float setpoint) {
        for(int i = 0; i < MPC_HORIZON; i++) {
            reference[i] = setpoint;
        }
    }
    
    float computeControl(float currentState, float previousControl, PowerConfig power) {
        float optimalControl = previousControl;
        float minCost = 1e9;
        
        // Economic optimization considering power costs
        for(float u = max(MIN_CURRENT, previousControl - 20.0f); 
            u <= min(MAX_CURRENT, previousControl + 20.0f); 
            u += 5.0f) {
            
            float cost = evaluateCost(currentState, u, power);
            
            if(cost < minCost) {
                minCost = cost;
                optimalControl = u;
            }
        }
        
        return optimalControl;
    }
    
private:
    float evaluateCost(float currentState, float control, PowerConfig power) {
        float totalCost = 0.0f;
        float state = currentState;
        
        for(int k = 0; k < MPC_HORIZON; k++) {
            state = A_MATRIX * state + B_MATRIX * control;
            
            float trackingError = state - reference[k];
            totalCost += Q_WEIGHT * trackingError * trackingError;
            
            if(k > 0) {
                // Add economic cost based on power source
                float powerCost = control * 1.8 * (power.gridRatio * power.gridPrice + power.pvRatio * power.pvPrice) / 1000.0;
                totalCost += R_WEIGHT * control * control + powerCost * 10.0;
            }
        }
        
        float terminalError = state - reference[MPC_HORIZON-1];
        totalCost += S_WEIGHT * terminalError * terminalError;
        
        return totalCost;
    }
    
    float max(float a, float b) { return (a > b) ? a : b; }
    float min(float a, float b) { return (a < b) ? a : b; }
};

IndustrialMPC industrialMPC;

// ========== MQTT FUNCTIONS ==========
void setupMQTT() {
  mqttClient.setServer(mqttBroker, mqttPort);
  mqttClient.setCallback(mqttCallback);
  ensureMQTTConnected();
}

void ensureMQTTConnected() {
  if (!mqttClient.connected()) {
    Serial.print("🔌 Connecting to MQTT...");
    if (mqttClient.connect(mqttClientId)) {
      Serial.println("✅ Connected!");
      
      // Subscribe to control topics
      mqttClient.subscribe("pem/control");
      mqttClient.subscribe("pem/current");
      mqttClient.subscribe("matlab/control");
      mqttClient.subscribe("matlab/current");
      mqttClient.subscribe("web/control");
      mqttClient.subscribe("web/mpc/decision");
      
      // Subscribe to power control topics
      mqttClient.subscribe("power/source/selection");
      mqttClient.subscribe("power/blending/ratio");
      mqttClient.subscribe("power/mpc/decision");
      mqttClient.subscribe("power/grid/setpoint");
      
      Serial.println("📡 Subscribed to all control topics");
      
      // Publish connection status
      mqttClient.publish("arduino/status", "connected");
      mqttConnected = true;
      
    } else {
      Serial.print("❌ Failed, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" - Retrying in 5s");
      mqttConnected = false;
    }
  }
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String message;
  for (int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  
  Serial.print("📨 MQTT RX: ");
  Serial.print(topic);
  Serial.print(" - ");
  Serial.println(message);
  
  // Handle system control commands
  if (strcmp(topic, "pem/control") == 0 || 
      strcmp(topic, "matlab/control") == 0 || 
      strcmp(topic, "web/control") == 0) {
    
    if (message == "start") {
      if (systemState == STATE_WAITING || systemState == STATE_STOPPED) {
        systemState = STATE_STARTING;
        stateStartTime = millis();
        updateSystemStateOnHMI();
        Serial.println("🚀 Remote START command received");
      }
    } else if (message == "stop") {
      if (systemState == STATE_RUNNING || systemState == STATE_STARTING) {
        systemState = STATE_STOPPING;
        stateStartTime = millis();
        updateSystemStateOnHMI();
        Serial.println("🛑 Remote STOP command received");
      }
    }
  }
  
  // Handle current setpoints
  else if (strcmp(topic, "pem/current") == 0 || 
           strcmp(topic, "matlab/current") == 0) {
    float newCurrent = message.toFloat();
    if (newCurrent >= 50 && newCurrent <= 150) {
      appliedCurrent = newCurrent;
      Serial.print("🎯 Current set to: ");
      Serial.print(newCurrent);
      Serial.println("A");
    }
  }
  
  // Handle power source selection
  else if (strcmp(topic, "power/source/selection") == 0) {
    powerConfig.source = message;
    Serial.print("⚡ Power source: ");
    Serial.println(message);
    updatePowerRelays();
  }
  
  // Handle power blending
  else if (strcmp(topic, "power/blending/ratio") == 0) {
    float newRatio = message.toFloat();
    if (newRatio >= 0 && newRatio <= 1) {
      powerConfig.gridRatio = newRatio;
      powerConfig.pvRatio = 1.0 - newRatio;
      Serial.print("🔀 Power blend - Grid: ");
      Serial.print(powerConfig.gridRatio * 100);
      Serial.print("%, PV: ");
      Serial.print(powerConfig.pvRatio * 100);
      Serial.println("%");
      updatePowerRelays();
    }
  }
  
  // Handle MPC decisions from web
  else if (strcmp(topic, "web/mpc/decision") == 0 || 
           strcmp(topic, "power/mpc/decision") == 0) {
    try {
      StaticJsonDocument<256> doc;
      DeserializationError error = deserializeJson(doc, message);
      if (!error) {
        if (doc.containsKey("grid_ratio")) {
          powerConfig.gridRatio = doc["grid_ratio"];
          powerConfig.pvRatio = 1.0 - powerConfig.gridRatio;
          updatePowerRelays();
          Serial.println("🧠 Web MPC decision applied");
        }
        if (doc.containsKey("optimal_current")) {
          appliedCurrent = doc["optimal_current"];
          Serial.print("🎯 MPC optimal current: ");
          Serial.print(appliedCurrent);
          Serial.println("A");
        }
      }
    } catch (...) {
      Serial.println("❌ Error parsing MPC decision");
    }
  }
}

void sendMQTTData() {
  if (!mqttClient.connected()) {
    ensureMQTTConnected();
    return;
  }
  
  StaticJsonDocument<512> doc;
  
  // System state
  doc["prodRateSet"] = prodRateSetpoint;
  doc["mode"] = systemMode == MODE_AUTO ? "AUTO" : 
                systemMode == MODE_MANUAL ? "MANUAL" : 
                systemMode == MODE_MPC ? "MPC" : 
                systemMode == MODE_ECONOMIC ? "ECONOMIC" : "NONE";
  doc["state"] = (int)systemState;
  doc["battery"] = smoothBatteryVoltage();
  doc["gsm"] = gsmRssiRaw;
  doc["slider"] = (int)round(prodRateSetpoint);
  
  // Sensor readings
  doc["temperature"] = cellTemperature;
  doc["voltage"] = stackVoltage;
  doc["current"] = stackCurrent;
  doc["o2Purity"] = o2Purity;
  doc["appliedCurrent"] = appliedCurrent;
  doc["mpcControlActive"] = mpcControlActive;
  
  // Production data
  doc["h2ProductionRate"] = h2ProductionRate;
  doc["o2ProductionRate"] = o2ProductionRate;
  
  // Power configuration
  doc["power_source"] = powerConfig.source;
  doc["grid_ratio"] = powerConfig.gridRatio;
  doc["pv_ratio"] = powerConfig.pvRatio;
  doc["grid_available"] = powerConfig.gridAvailable;
  doc["pv_available"] = powerConfig.pvAvailable;
  
  // Timestamp
  DateTime now = rtc.now(); 
  char dateStr[64]; 
  snprintf(dateStr, sizeof(dateStr), "%02d-%02d-%04d %02d:%02d:%02d", 
           now.day(), now.month(), now.year(), now.hour(), now.minute(), now.second()); 
  doc["rtcDateTime"] = dateStr;
  
  char buffer[512];
  serializeJson(doc, buffer);
  
  mqttClient.publish("arduino/sensors", buffer);
  
  // Individual values for easy parsing
  mqttClient.publish("arduino/current", String(appliedCurrent).c_str());
  mqttClient.publish("arduino/temperature", String(cellTemperature).c_str());
  mqttClient.publish("arduino/prodRate", String(prodRateSetpoint).c_str());
  mqttClient.publish("arduino/power/source", powerConfig.source.c_str());
  mqttClient.publish("arduino/power/grid_ratio", String(powerConfig.gridRatio).c_str());
  
  Serial.print("📤 MQTT TX: ");
  Serial.println(buffer);
}

// ========== POWER RELAY CONTROL ==========
void updatePowerRelays() {
  // Control grid relay
  if (powerConfig.gridRatio > 0.1 && powerConfig.gridAvailable) {
    digitalWrite(GRID_RELAY, HIGH);
  } else {
    digitalWrite(GRID_RELAY, LOW);
  }
  
  // Control PV relay
  if (powerConfig.pvRatio > 0.1 && powerConfig.pvAvailable) {
    digitalWrite(PV_RELAY, HIGH);
  } else {
    digitalWrite(PV_RELAY, LOW);
  }
  
  Serial.print("🔌 Power relays - Grid: ");
  Serial.print(digitalRead(GRID_RELAY) ? "ON" : "OFF");
  Serial.print(", PV: ");
  Serial.println(digitalRead(PV_RELAY) ? "ON" : "OFF");
}

// ========== NEXTION HMI FUNCTIONS ==========
void sendToNextion(const char *fmt, ...) {
  char buf[300];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);
  nexSerial.print(buf);
  nexSerial.write(0xFF); nexSerial.write(0xFF); nexSerial.write(0xFF);
}

void setText(const char *component, const char *txt) {
  char buf[340]; snprintf(buf, sizeof(buf), "%s.txt=\"%s\"", component, txt);
  sendToNextion("%s", buf);
}

void setText(const char *component, String s) { setText(component, s.c_str()); }

// ========== BATTERY MONITORING ==========
float readBatteryVoltage() { 
  float rawVoltage = (float)analogRead(BAT_PIN) / ((1 << ADC_BITS) - 1) * ADC_REF_VOLTAGE;
  return rawVoltage * VOLTAGE_DIVIDER_FACTOR;
}

float smoothBatteryVoltage() {
  static float lastBatt = 3.8f;
  float newV = readBatteryVoltage();
  lastBatt = 0.9f * lastBatt + 0.1f * newV;
  return lastBatt;
}

int batteryPercentFromVoltage(float v) {
  float pct = (v - 0.3f) / (0.44f - 0.3f) * 100.0f;
  return constrain((int)round(pct), 0, 100);
}

void updateBatteryOnHMI() {
  float v = smoothBatteryVoltage();
  int pct = batteryPercentFromVoltage(v);
  sendToNextion("j8.val=%d", pct);
}

// ========== SENSOR READING FUNCTIONS ==========
float readCellTemperature() {
  // Simulate temperature reading - replace with actual sensor
  float temp = 65.0f + (random(-50, 50) / 10.0f);
  return constrain(temp, 60.0f, 80.0f);
}

float readStackVoltage() {
  // Simulate voltage reading - replace with actual sensor
  float voltage = 38.0f + (random(-20, 20) / 10.0f);
  return constrain(voltage, 35.0f, 42.0f);
}

float readStackCurrent() {
  // Simulate current reading - replace with actual sensor
  return appliedCurrent + (random(-10, 10) / 10.0f);
}

float readO2Purity() {
  // Simulate O2 purity reading - replace with actual sensor
  float purity = 99.5f + (random(-5, 5) / 10.0f);
  return constrain(purity, 99.0f, 100.0f);
}

void readAllSensors() {
  cellTemperature = readCellTemperature();
  stackVoltage = readStackVoltage();
  stackCurrent = readStackCurrent();
  o2Purity = readO2Purity();
  
  // Calculate production rates based on current
  h2ProductionRate = appliedCurrent * 0.00042f;
  o2ProductionRate = appliedCurrent * 0.00021f;
}

// ========== GSM FUNCTIONS ==========
void updateGsmSignal() {
  simSerial.println("AT+CSQ");
  unsigned long t0 = millis(); String resp = "";
  while (millis() - t0 < 500) while (simSerial.available()) resp += (char)simSerial.read();
  int idx = resp.indexOf("+CSQ:");
  if (idx >= 0) {
    int comma = resp.indexOf(",", idx);
    if (comma >= 0) gsmRssiRaw = resp.substring(idx + 6, comma).toInt();
  } else gsmRssiRaw = 99;
  
  int bars = (gsmRssiRaw == 99) ? 0 : (gsmRssiRaw <=5?1:(gsmRssiRaw<=10?2:(gsmRssiRaw<=20?3:4)));
  for (int i=4;i<=7;++i){
    int idxFromBottom = 7-i+1;
    if(bars>=idxFromBottom){
      sendToNextion("j%d.bco=2016",i); 
      sendToNextion("j%d.val=100",i);
    } else {
      sendToNextion("j%d.bco=65504",i); 
      sendToNextion("j%d.val=0",i);
    } 
  }
  gsmOK = (gsmRssiRaw!=99);
}

// ========== WIFI FUNCTIONS ==========
void ensureWiFiConnected() {
  if(WiFi.status()!=WL_CONNECTED){
    WiFi.begin(ssid,password); 
    unsigned long start=millis(); 
    while(WiFi.status()!=WL_CONNECTED && millis()-start<4000) delay(200);
  } 
  wifiOK=(WiFi.status()==WL_CONNECTED);
}

void updateWiFiVisual() {
  if(!wifiOK){
    for(int i=0;i<=4;i++) sendToNextion("vis p%d,0",i); 
    sendToNextion("vis p0,1"); 
    return;
  }
  
  long rssi = WiFi.RSSI();
  int level;
  if (rssi > -50) level = 4;
  else if (rssi > -60) level = 3;
  else if (rssi > -70) level = 2;
  else if (rssi > -80) level = 1;
  else level = 0;
  
  for(int i=0;i<=4;i++) sendToNextion("vis p%d,%d",i,(i<=level)?1:0);
}

// ========== SYSTEM STATE DISPLAY ==========
void updateSystemStateOnHMI() {
  ensureWiFiConnected();
  updateWiFiVisual();
  updateGsmSignal();
  float battV = smoothBatteryVoltage();
  bool powerOK = (battV>0.3f);
  
  String status;
  switch(systemState){
    case STATE_WAITING: status="🔄 WAITING"; break;
    case STATE_STARTING: status="🚀 STARTING"; break;
    case STATE_RUNNING: 
      if(wifiOK && gsmOK && powerOK) {
        status = "✅ RUNNING:";
        switch(systemMode) {
          case MODE_MPC: status += "MPC"; break;
          case MODE_ECONOMIC: status += "ECO"; break;
          case MODE_AUTO: status += "AUTO"; break;
          case MODE_MANUAL: status += "MANUAL"; break;
          default: status += "GOOD"; break;
        }
      } else {
        status="⚠️ RUNNING:BAD";
      }
      break;
    case STATE_STOPPING: status="🛑 STOPPING"; break;
    case STATE_STOPPED: status="⏹️ STOPPED"; break;
  }
  
  setText("t7",status);
  
  // Update mode indicator
  String modeText;
  switch(systemMode) {
    case MODE_MPC: modeText = "🧠 MPC CONTROL"; break;
    case MODE_ECONOMIC: modeText = "💰 ECONOMIC MPC"; break;
    case MODE_AUTO: modeText = "🔧 AUTO MODE"; break;
    case MODE_MANUAL: modeText = "👤 MANUAL MODE"; break;
    default: modeText = "❌ NO MODE"; break;
  }
  setText("t9", modeText);
  
  // Update power source display
  String powerText = "⚡ " + powerConfig.source + " G:" + String(int(powerConfig.gridRatio*100)) + "% P:" + String(int(powerConfig.pvRatio*100)) + "%";
  setText("t12", powerText);
  
  // Update production rates on HMI
  setText("t10", String(h2ProductionRate * 3600, 1) + " L/h");
  setText("t11", String(o2ProductionRate * 3600, 1) + " L/h");
  
  Serial.print("📺 HMI status: ");
  Serial.println(status);
}

// ========== MPC CONTROL ==========
float runIndustrialMPC() {
    if(systemMode != MODE_MPC && systemMode != MODE_ECONOMIC) {
        return appliedCurrent;
    }
    
    float productionReference = prodRateSetpoint / 100.0f * 0.05f;
    industrialMPC.setReference(productionReference);
    
    float currentState = h2ProductionRate;
    float optimalControl = industrialMPC.computeControl(currentState, appliedCurrent, powerConfig);
    
    Serial.print("🎯 Industrial MPC: ");
    Serial.print("State="); Serial.print(currentState, 4);
    Serial.print(", Ref="); Serial.print(productionReference, 4);
    Serial.print(", Control="); Serial.print(optimalControl, 1);
    Serial.println("A");
    
    return optimalControl;
}

// ========== SAFETY FUNCTIONS ==========
void enforceSafetyConstraints() {
  float safeCurrent = appliedCurrent;
  
  // Temperature safety
  if (cellTemperature > 75.0f) {
    safeCurrent = min(safeCurrent, 150.0f);
    Serial.println("🛡️ Temperature derating: 150A max");
  }
  if (cellTemperature > MAX_TEMPERATURE - 2.0f) {
    safeCurrent = min(safeCurrent, 120.0f);
    Serial.println("🛡️ Critical temperature: 120A max");
  }
  
  // Voltage safety
  if (stackVoltage > MAX_VOLTAGE - 2.0f) {
    safeCurrent = min(safeCurrent, 160.0f);
    Serial.println("🛡️ High voltage: 160A max");
  }
  
  // Purity safety
  if (o2Purity < MIN_O2_PURITY + 0.2f) {
    safeCurrent = min(safeCurrent, 140.0f);
    Serial.println("🛡️ Low purity: 140A max");
  }
  
  // Absolute limits
  safeCurrent = constrain(safeCurrent, MIN_CURRENT, MAX_CURRENT);
  
  if (abs(safeCurrent - appliedCurrent) > 5.0f) {
    Serial.print("🛡️ Safety override: ");
    Serial.print(appliedCurrent);
    Serial.print("A -> ");
    Serial.print(safeCurrent);
    Serial.println("A");
    appliedCurrent = safeCurrent;
  }
}

void performSafetyCheck() {
  readAllSensors();
  enforceSafetyConstraints();
  
  // Emergency shutdown conditions
  if (cellTemperature > MAX_TEMPERATURE || o2Purity < MIN_O2_PURITY) {
    Serial.println("🚨 CRITICAL: Emergency shutdown!");
    systemState = STATE_STOPPING;
    stateStartTime = millis();
    appliedCurrent = 0.0f;
    digitalWrite(GRID_RELAY, LOW);
    digitalWrite(PV_RELAY, LOW);
  }
}

// ========== CURRENT CONTROL ==========
void applyCurrent(float current) {
  current = constrain(current, MIN_CURRENT, MAX_CURRENT);
  appliedCurrent = current;
  
  Serial.print("🔧 Applying current: ");
  Serial.print(current);
  Serial.println("A");
  
  // Update HMI with applied current
  sendToNextion("h0.val=%d", (int)round(current));
}

// ========== NEXTION INPUT HANDLING ==========
int extractFirstInt(const String &s){String num="";for(unsigned int i=0;i<s.length();++i){char c=s.charAt(i);if(isDigit(c)||(c=='-'&&num.length()==0))num+=c;else if(num.length())break;}return num.length()?num.toInt():0;}

void handleNextionInput(){
  static String buf=""; static uint8_t ffCount=0;
  while(nexSerial.available()){
    char c=nexSerial.read();
    if((uint8_t)c==0xFF){ffCount++; if(ffCount>=3){String proc=buf; buf=""; ffCount=0; proc.trim(); if(proc.length()==0) continue;
      int pos=0; while(pos<proc.length()){int startB=proc.indexOf('b',pos); int startH=proc.indexOf('h',pos); int start=startB; if(startH>=0&&(startH<start||start<0)) start=startH; if(start<0) break;
        int nextB=proc.indexOf('b',start+1); int nextH=proc.indexOf('h',start+1); int next=-1; if(nextB>=0 && nextH>=0) next=min(nextB,nextH); else if(nextB>=0) next=nextB; else if(nextH>=0) next=nextH;
        String cmd=(next>=0)?proc.substring(start,next):proc.substring(start); cmd.trim(); pos=(next>=0)?next:proc.length(); if(cmd.length()==0) continue;
        
        if(cmd.startsWith("b0=")){if(systemState==STATE_WAITING||systemState==STATE_STOPPED){systemState=STATE_STARTING; stateStartTime=millis(); updateSystemStateOnHMI(); Serial.println("[HMI] START");}}
        else if(cmd.startsWith("b1=")){if(systemState==STATE_RUNNING||systemState==STATE_STARTING){systemState=STATE_STOPPING; stateStartTime=millis(); updateSystemStateOnHMI(); Serial.println("[HMI] STOP");}}
        else if(cmd.startsWith("b2=")){systemMode=MODE_AUTO; if(systemState==STATE_STARTING||systemState==STATE_WAITING||systemState==STATE_STOPPED) systemState=STATE_RUNNING; updateSystemStateOnHMI();}
        else if(cmd.startsWith("b3=")){systemMode=MODE_MANUAL; if(systemState==STATE_STARTING||systemState==STATE_WAITING||systemState==STATE_STOPPED) systemState=STATE_RUNNING; updateSystemStateOnHMI();}
        else if(cmd.startsWith("b4=")){systemMode=MODE_MPC; mpcControlActive=true; if(systemState==STATE_STARTING||systemState==STATE_WAITING||systemState==STATE_STOPPED) systemState=STATE_RUNNING; updateSystemStateOnHMI(); Serial.println("[HMI] MPC MODE");}
        else if(cmd.startsWith("b5=")){systemMode=MODE_ECONOMIC; mpcControlActive=true; if(systemState==STATE_STARTING||systemState==STATE_WAITING||systemState==STATE_STOPPED) systemState=STATE_RUNNING; updateSystemStateOnHMI(); Serial.println("[HMI] ECONOMIC MPC MODE");}
        else if(cmd.startsWith("h0=") || cmd.startsWith("h0.val=")){
          Serial.print("[RAW SLIDER CMD] "); Serial.println(cmd);
          int value=extractFirstInt(cmd); 
          Serial.print("[PARSED SLIDER] "); Serial.println(value);
          value=constrain(value,0,100); 
          prodRateSetpoint=(float)value; 
          sliderMoved=true; 
          
          // Convert slider to current (0-100% -> 100-200A)
          manualCurrentSetpoint = map(value, 0, 100, 100, 200);
          
          Serial.print("🎚️ Slider: ");
          Serial.print(value);
          Serial.print("% -> ");
          Serial.print(manualCurrentSetpoint);
          Serial.println("A");
          
          if(systemMode == MODE_MANUAL) {
            applyCurrent(manualCurrentSetpoint);
          }
          
          // Send MQTT update
          if(mqttConnected){
            StaticJsonDocument<128> j; 
            j["prodRate"]=value;
            j["manualCurrent"]=manualCurrentSetpoint;
            char b[128]; serializeJson(j,b); 
            mqttClient.publish("arduino/slider", b);
            Serial.print("📤 MQTT slider update: "); Serial.println(b);
          }
        }
      }
    }} else buf+=c;
  }
}

// ========== CONTROL LOGIC ==========
void handleMPCControl() {
  if((systemMode == MODE_MPC || systemMode == MODE_ECONOMIC) && mpcControlActive) {
    float mpcCurrent = runIndustrialMPC();
    applyCurrent(mpcCurrent);
    performSafetyCheck();
  } else if(systemMode == MODE_MANUAL) {
    applyCurrent(manualCurrentSetpoint);
    performSafetyCheck();
  } else if(systemMode == MODE_AUTO) {
    float autoCurrent = 150.0f;
    applyCurrent(autoCurrent);
    performSafetyCheck();
  }
}

// ========== SETUP ==========
void setup() {
  Serial.begin(115200);
  nexSerial.begin(9600);
  simSerial.begin(9600);
  Wire.begin();
  
  // Initialize relays
  pinMode(GRID_RELAY, OUTPUT);
  pinMode(PV_RELAY, OUTPUT);
  digitalWrite(GRID_RELAY, LOW);
  digitalWrite(PV_RELAY, LOW);
  
  analogReadResolution(ADC_BITS);
  randomSeed(analogRead(A0) ^ micros());
    Serial.println("\n🏭 INDUSTRIAL PEM ELECTROLYZER CONTROLLER");
  Serial.println("==========================================");

  // RTC Initialization
  Serial.println("⏰ RTC Initialization...");
  if (!rtc.begin()) {
    Serial.println("❌ RTC not found!");
  } else {
    DateTime nowRTC = rtc.now();
    if (nowRTC.year() < 2024 || nowRTC.year() > 2099) {
      rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
      Serial.println("🕒 RTC adjusted from compile time");
    } else {
      Serial.print("✅ RTC running: ");
      Serial.print(nowRTC.day()); Serial.print("/");
      Serial.print(nowRTC.month()); Serial.print("/");
      Serial.print(nowRTC.year()); Serial.print(" ");
      Serial.print(nowRTC.hour()); Serial.print(":");
      Serial.print(nowRTC.minute()); Serial.print(":");
      Serial.println(nowRTC.second());
    }
  }

  // Initialize HMI
  Serial.println("📺 Initializing HMI...");
  setText("t7", "🔄 INITIALIZING");
  setText("t5", "");
  setText("t0", "--");
  setText("t1", "--");
  setText("t2", "--");
  setText("t3", "--");
  setText("t4", "--");
  setText("t9", "NO MODE");
  setText("t10", "0.0 L/h");
  setText("t11", "0.0 L/h");
  setText("t12", "⚡ INITIALIZING");
  sendToNextion("h0.val=%d", (int)round(prodRateSetpoint));

  // Network initialization
  Serial.println("🌐 Network Initialization...");
  ensureWiFiConnected();
  updateWiFiVisual();
  updateGsmSignal();
  updateBatteryOnHMI();
  
  // MQTT Setup
  Serial.println("📡 MQTT Setup...");
  setupMQTT();

  // Final initialization
  updateSystemStateOnHMI();
  lastFullUpdate = lastMqttUpdate = lastSafetyCheck = lastMPCUpdate = millis();
  
  Serial.println("✅ INDUSTRIAL CONTROLLER READY");
  Serial.println("   ⚡ Dual Power Source Control");
  Serial.println("   🌐 3-Way MQTT Communication");
  Serial.println("   🧠 Advanced MPC Integration");
  Serial.println("   🛡️ Comprehensive Safety Systems");
}

// ========== MAIN LOOP ==========
void loop(){
  unsigned long now = millis();
  
  // Handle MQTT communication
  mqttClient.loop();
  
  // Process HMI inputs
  handleNextionInput();

  // MPC control updates (10 Hz)
  if(now - lastMPCUpdate >= MPC_UPDATE_MS){
    lastMPCUpdate = now;
    handleMPCControl();
  }

  // Safety checks (2 Hz)
  if(now - lastSafetyCheck >= SAFETY_CHECK_MS){
    lastSafetyCheck = now;
    performSafetyCheck();
  }

  // Update time and status display (0.2 Hz)
  if(now - lastFullUpdate >= FULL_UPDATE_MS){
    lastFullUpdate = now; 
    DateTime dt = rtc.now();
    
    // Update time display
    char buf[32]; 
    snprintf(buf, sizeof(buf), "%02d", dt.hour()); 
    setText("t0", buf); 
    snprintf(buf, sizeof(buf), "%02d", dt.minute()); 
    setText("t1", buf);
    snprintf(buf, sizeof(buf), "%02d", dt.second()); 
    setText("t2", buf); 
    
    char dateStr[32]; 
    snprintf(dateStr, sizeof(dateStr), "%02d %s %04d", 
             dt.day(), 
             (const char*[]){"Jan","Feb","Mar","Apr","May","Jun","Jul","Aug","Sep","Oct","Nov","Dec"}[dt.month()-1], 
             dt.year()); 
    setText("t3", dateStr);
    
    char upd[16]; 
    snprintf(upd, sizeof(upd), "%02d:%02d:%02d", dt.hour(), dt.minute(), dt.second()); 
    setText("t8", upd);
    
    updateBatteryOnHMI(); 
    updateSystemStateOnHMI();
  }

  // State machine transitions
  if(systemState == STATE_STARTING && now - stateStartTime >= START_DELAY){
    if(systemMode == MODE_AUTO || systemMode == MODE_MANUAL || systemMode == MODE_MPC || systemMode == MODE_ECONOMIC){
      systemState = STATE_RUNNING; 
      updateSystemStateOnHMI(); 
      Serial.println("[SYSTEM] STARTING -> RUNNING");
    }
  }
  else if(systemState == STATE_STOPPING && now - stateStartTime >= STOP_DELAY){
    systemState = STATE_STOPPED; 
    systemMode = MODE_NONE; 
    mpcControlActive = false;
    appliedCurrent = 0.0f;
    digitalWrite(GRID_RELAY, LOW);
    digitalWrite(PV_RELAY, LOW);
    updateSystemStateOnHMI(); 
    Serial.println("[SYSTEM] STOPPING -> STOPPED");
  }

  // MQTT data transmission (1 Hz)
  if(now - lastMqttUpdate >= MQTT_UPDATE_MS){
    lastMqttUpdate = now; 
    sendMQTTData();
  }

  delay(50); // 20 Hz main loop
}

  
