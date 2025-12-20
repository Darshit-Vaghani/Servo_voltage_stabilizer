// ESP32 version of your code (uses interrupts for frequency & phase measurement)
// Pin mapping (change if needed):
// V_FREQ_PIN  -> GPIO16 (rising edge from voltage zero crossing detector)
// I_FREQ_PIN  -> GPIO17 (rising edge from current zero crossing detector)
// V_IN_ADC_PIN   -> GPIO34 (analog input for voltage measurement)
// I_IN_ADC_PIN   -> GPIO35 (analog input for current measurement)

#include <Arduino.h>

const int V_IN_FREQ_PIN = 22;
const int I_IN_FREQ_PIN = 23;
const int V_IN_ADC_PIN = 36;
const int I_IN_ADC_PIN = 39;
int ref = 2500;
int set_point = 230;
byte dec = 18;
byte inc = 17;
float error = 0;
float v_in, v_out;


const int V_OUT_FREQ_PIN = 19;
const int I_OUT_FREQ_PIN = 21;
const int V_OUT_ADC_PIN = 34;
const int I_OUT_ADC_PIN = 35;

volatile unsigned long lastVmicros = 0;
volatile unsigned long lastImicros = 0;
volatile unsigned long periodV = 0;      // microseconds between V rising edges
volatile unsigned long periodI = 0;      // microseconds between I rising edges
volatile unsigned long phaseDiffVI = 0;  // microseconds from V edge to I edge (signed-ish)

portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

double time1, time2;

double voltage_frequncy(byte pin) {
  double time = 0;
  time = pulseIn(pin, HIGH);
  time = time * 2;
  time = time / 1000000;
  time = 1 / time;
  //Serial.println("Voltage Frequncy = " +(String)time);
  return time;
}

float current_frequncy(byte pin) {
  double time = 0;
  time = pulseIn(pin, HIGH);
  time = time * 2;
  time = time / 1000000;
  time = 1 / time;
  //Serial.println("Current Frequncy = " +(String)time);
  return (float)time;
}


float pf(byte v_pin, byte i_pin) {

  while (!(digitalRead(v_pin)) == HIGH && digitalRead(i_pin) == HIGH)
    ;
  time1 = micros();
  while (digitalRead(v_pin) == HIGH && digitalRead(i_pin) == HIGH)
    ;
  time2 = micros();
  time1 = time2 - time1;
  time1 = time1 / 1000000.0;
  float phi = 2.0 * 3.1428 * 50.0 * time1;
  float power_factor = cos(phi);

  if (power_factor > 1) {
    power_factor = 1.0;
  }
  //Serial.println("Power Factor = "+(String)power_factor);
  return power_factor;
}

float voltage_rms_sample(byte pin) {
  // ADC: ESP32 default ADC resolution is 12-bit (0..4095), Vref ~ 3.3V
  const int samples = 1000;
  double sumSquares = 0.0;
  double vAdc;
  // ensure ADC settings
  analogReadResolution(12);                // 0-4095
  analogSetPinAttenuation(pin, ADC_11db);  // support ~0-3.3V

  for (int i = 0; i < samples; i++) {
    int adcValue = analogRead(pin);
    vAdc = (adcValue * 3.3) / 4095.0;  // convert to volts (0..3.3)
    //Serial.println(vAdc,3);
    // your offset used earlier was ~1.676 (for 5V ADC). For ESP32 supply it's ~1.65
    double acSignal = vAdc - 1.567;                 // Remove DC offset (adjust if needed)
    double actualVoltage = acSignal / 0.000202020;  // Reverse gain (keep your gain)
    sumSquares += actualVoltage * actualVoltage;
    //delayMicroseconds(200);  // sampling delay (for 50Hz ~ 200us per sample)
  }
  double rmsVoltage = sqrt(sumSquares / (double)samples);
  //Serial.print("AC RMS Voltage = ");
  //Serial.print(rmsVoltage, 3);
  //Serial.println(" V");
  return (float)rmsVoltage;
}

float current_rms_sample(byte pin) {
  const int samples = 1000;
  double sumSquares = 0.0;
  double vAdc;
  analogReadResolution(12);
  analogSetPinAttenuation(pin, ADC_11db);

  for (int i = 0; i < samples; i++) {
    int adcValue = analogRead(pin);
    vAdc = (adcValue * 3.3) / 4095.0;

    double vAc = vAdc - 1.567;  // remove DC offset (adjust if needed)
    // keep your CT scaling (you used vAc/33 then *1000)
    double iSecondary = vAc / 33.0;      // as in your code -> Ampere (secondary)
    double iPrimary = iSecondary * ref;  // convert according to CT ratio
    sumSquares += iPrimary * iPrimary;
    delayMicroseconds(200);
  }

  double iRms = sqrt(sumSquares / (double)samples);
  //Serial.print("AC RMS Current = ");
  //Serial.print(iRms, 3);
  //Serial.println(" A");
  return (float)iRms;
}

void setup() {
  Serial.begin(115200);
  delay(100);

  pinMode(V_IN_FREQ_PIN, INPUT_PULLUP);
  pinMode(I_IN_FREQ_PIN, INPUT_PULLUP);
  pinMode(V_OUT_FREQ_PIN, INPUT_PULLUP);
  pinMode(I_OUT_FREQ_PIN, INPUT_PULLUP);

  // ADC pins are input-only on 34,35 so no pinMode() required, but it doesn't hurt:
  pinMode(V_IN_ADC_PIN, INPUT);
  pinMode(I_IN_ADC_PIN, INPUT);
  pinMode(V_OUT_ADC_PIN, INPUT);
  pinMode(I_OUT_ADC_PIN, INPUT);
  pinMode(inc, OUTPUT);
  pinMode(dec, OUTPUT);
  digitalWrite(inc, LOW);
  digitalWrite(dec, LOW);


  // ADC config
  analogReadResolution(12);
  analogSetPinAttenuation(V_IN_ADC_PIN, ADC_11db);
  analogSetPinAttenuation(I_IN_ADC_PIN, ADC_11db);
  analogSetPinAttenuation(V_OUT_ADC_PIN, ADC_11db);
  analogSetPinAttenuation(I_OUT_ADC_PIN, ADC_11db);

  Serial.println("ESP32 measurement started");
}

void loop() {
  //if(Serial.available()) {
  // ref = (Serial.readString()).toInt();
  // Serial.println("The Ref is set = "+(String)ref);
  //}

  if(Serial.available()) {
    set_point = (Serial.readString()).toInt();
    Serial.println("The set Target is "+(String)set_point);
  }
  v_in = voltage_rms_sample(V_IN_ADC_PIN);
  //float c_in = current_rms_sample(I_IN_ADC_PIN);
  v_out = voltage_rms_sample(V_OUT_ADC_PIN);

if(v_in < 40) v_in =0;
if(v_out < 40) v_out =0;
  //----------------------algo

  error = v_out - set_point;

  digitalWrite(inc, LOW);
  digitalWrite(dec, LOW);

  if (error < (-5)) {
    
    while (error < (-5)) {
      digitalWrite(inc, HIGH);
      v_out = voltage_rms_sample(V_OUT_ADC_PIN);
      error = v_out - set_point;
    }
    digitalWrite(inc, LOW);
  }

  else if (error > 5) {
    while (error > 5) {
      digitalWrite(dec, HIGH);
      v_out = voltage_rms_sample(V_OUT_ADC_PIN);
      error = v_out - set_point;
    }
    digitalWrite(dec, LOW);
  }

  //--------------------------------

  //float c_out = current_rms_sample(I_OUT_ADC_PIN);
  //float vf = voltage_frequncy(V_IN_FREQ_PIN);
  //float If = current_frequncy(I_IN_FREQ_PIN);
  //float p_f_in = pf(V_IN_FREQ_PIN,I_IN_FREQ_PIN);
  //float p_f_out = pf(V_OUT_FREQ_PIN,I_OUT_FREQ_PIN);
  //float power_in = v_in * c_in * p_f_in;
  //float power_out = v_out * c_out * p_f_out;
  //Serial.println("Input Voltage = "+(String)v_in +"V");
  //Serial.println("Output Voltage = "+(String)v_out +"V");
  //Serial.println("Input current = "+(String)c_in+"A");
  //Serial.println("OUTPUT Voltage = "+(String)v_out +"V");
  //Serial.println("OUTPUT current = "+(String)c_out+"A");
  //Serial.println("Voltage Frequncy = "+(String)vf + "Hz");
  //Serial.println("Current Frequncy = "+(String)If + "Hz");
  //Serial.println("Input Power = " +(String)power_in + "W");
  //Serial.println("OUTPUT Power = " +(String)power_out + "W");
  //Serial.println("%n = "+ (String)((float)(power_out/power_in)));
  //Serial.println("-----------------------------------------------------------------------------");
  //delay(1000);
}
