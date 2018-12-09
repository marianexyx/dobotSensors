#include <OneWire.h>
#include <DallasTemperature.h>

const int ACS712_30A_PIN = A1;

const int THERMOMETERS_PIN = 9;

const int EMERGENCY_STOP_PIN = 2;
const bool EMERGENCY_STOP_PRESSED = LOW;
const bool EMERGENCY_STOP_REALESED = HIGH;

const int RELAY_PIN = 8;
const bool POWER_SUPPLY_DC_ON = HIGH;
const bool POWER_SUPPLY_DC_OFF = LOW;

const int DUST_ANALOG_PIN = A0;
const int DUST_LED_PIN = 7;

const float MAX_DUST = 350.f; //[µg/m3]
const float MAX_CURRENT = 4.f; //[A]
const float MAX_TEMP = 26.f; //[°C]


OneWire thermometersWire(THERMOMETERS_PIN);
DallasTemperature thermometers(&thermometersWire);

bool bPowerSupplyDC;
bool bEmergencyStopWasPressed;

void setup()
{
  //EXTERNAL: 0-5V, musi być podpięte zewnętrzne napięcie pod AREF pin i trzeba to wywołać przed jakimkolkwiek analogRead()
  analogReference(EXTERNAL);

  thermometers.begin();
  pinMode(EMERGENCY_STOP_PIN, INPUT);
  pinMode(RELAY_PIN, OUTPUT);
  pinMode(ACS712_30A_PIN, INPUT);
  digitalWrite(RELAY_PIN, LOW);

  pinMode(DUST_LED_PIN, OUTPUT);
  digitalWrite(DUST_LED_PIN, LOW);

  bPowerSupplyDC = POWER_SUPPLY_DC_ON;
  bEmergencyStopWasPressed = false;

  Serial.begin(9600);
}

void loop()
{
  bool bEmergencyButton = digitalRead(EMERGENCY_STOP_PIN);
  float fDust = computeAvgDust();
  float fCurrent = measureAvgCurrent();
  thermometers.requestTemperatures(); //zbyt częsty odczyt będzie podgrzewał termometry
  float fTemp1 = thermometers.getTempCByIndex(0);
  float fTemp2 = thermometers.getTempCByIndex(1);

  /*Serial.print("Dust = "); Serial.print(fDust); Serial.print(" [µg/m3]");
  Serial.print(". Current = "); Serial.print(fCurrent); Serial.print(" [A]");
  Serial.print(". 1st temp = "); Serial.print(fTemp1); Serial.print(" [°C]");
  Serial.print(", 2nd temp = "); Serial.print(fTemp2); Serial.println(" [°C]");*/

  Serial.print("@{\"stopBtn\":"); Serial.print(!bEmergencyButton); Serial.print(", "); //buton state is high when not pressed
  Serial.print("\"dust\":"); Serial.print(fDust); Serial.print(", ");
  Serial.print("\"current\":"); Serial.print(fCurrent); Serial.print(", ");
  Serial.print("\"temp1\":"); Serial.print(fTemp1); Serial.print(", ");
  Serial.print("\"temp2\":"); Serial.print(fTemp2); Serial.print("}$");

  if (bEmergencyButton == EMERGENCY_STOP_PRESSED)
  {
    bEmergencyStopWasPressed = true;
    bPowerSupplyDC = POWER_SUPPLY_DC_OFF;
  }
  else if (fDust > MAX_DUST || fCurrent > MAX_CURRENT || fTemp1 > MAX_TEMP || fTemp2 > MAX_TEMP)
  {
    bPowerSupplyDC = POWER_SUPPLY_DC_OFF;
  }
  else if (bEmergencyStopWasPressed && bEmergencyButton == EMERGENCY_STOP_REALESED)
  {
    bPowerSupplyDC = POWER_SUPPLY_DC_ON;
    bEmergencyStopWasPressed = false;
  }

  //do not manage current flow with arduino
  /*if (bPowerSupplyDC == POWER_SUPPLY_DC_OFF) 
    digitalWrite(RELAY_PIN, POWER_SUPPLY_DC_OFF);
  else digitalWrite(RELAY_PIN, POWER_SUPPLY_DC_ON);*/
}

float measureAvgCurrent()
{
  const float dVoltPerAnalogPoint = 0.0048828125; //5V/1024 punktów odczytu na analogu
  const float voltPerAmper = 0.066; // dla modułu 20A - 100, dla modułu 30A - 66
  int rawAnalog = 0;
  float voltage = 0;
  double current = 0;
  double avgCurrent = 0;
  for (int i = 0; i < 100; i++)
  {
    rawAnalog = analogRead(ACS712_30A_PIN) - 11; // 512 powinno być po środku (na zerze)
    voltage = (rawAnalog * dVoltPerAnalogPoint) - 2.5; //2.5 jest po środku
    current = voltage / voltPerAmper; // natężenie w A

    avgCurrent += current;
    delay(10);
  }
  avgCurrent /= 100;
  return avgCurrent;
}

float computeDust()
{
  const int DUST_MIN_MILI_VOLTAGE = 600; //mv - próg dolnego zakresu napięcia dla braku pyłu
  int dustAdcVal;
  float dustMiliVoltage;
  float dustMiliVoltageDivider;
  float dustDensity;

  // Blyskamy IR, czekamy 280ms, odczytujemy napiecie ADC
  digitalWrite(DUST_LED_PIN, HIGH);
  delayMicroseconds(280);
  dustAdcVal = analogRead(DUST_ANALOG_PIN);
  digitalWrite(DUST_LED_PIN, LOW);

  dustMiliVoltage = (5000.0 / 1024.0) * dustAdcVal;
  dustMiliVoltageDivider = dustMiliVoltage * 11;
  dustDensity = (dustMiliVoltageDivider - DUST_MIN_MILI_VOLTAGE) * 0.2;

  /*Serial.print("computeDust(): dustAdcVal = "); Serial.print(dustAdcVal);
    Serial.print(", dustMiliVoltage = "); Serial.print(dustMiliVoltage);
    Serial.print(", dustMiliVoltageDivider = "); Serial.print(dustMiliVoltageDivider);
    Serial.print(", dustVal = "); Serial.println(dustDensity);*/

  // Obliczamy zanieczyszczenie jesli zmierzone napiecie ponad prog
  if (dustMiliVoltageDivider > DUST_MIN_MILI_VOLTAGE)
    return dustDensity;
  else return 0;
}

float computeAvgDust()
{
  float fVal = 0;
  float fAvgVal = 0;
  int nMeasurements = 0;
  const int MAX_ITERS = 10;

  while (MAX_ITERS > nMeasurements)
  {
    fVal = computeDust();
    if (fVal > 0) //Do sredniej liczmy tylko prawidlowe pomiary
      fAvgVal += fVal;
    delay(50);
    nMeasurements++;
  }
  fAvgVal /= MAX_ITERS;

  return fAvgVal;
}

