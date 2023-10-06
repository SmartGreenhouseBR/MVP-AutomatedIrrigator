#include <Arduino.h>
#include "WiFi.h"
#include "PubSubClient.h"
#include "Adafruit_Sensor.h"
#include "Adafruit_TSL2561_U.h"
#include "DHT.h"

#define DHTPIN 4
#define SENSOR 35
#define RELE 16
#define SOLO_SENSOR 34
#define COOLER 17
#define LED 27
#define DHTTYPE    DHT11

volatile byte pulseCount = 0;


WiFiClient espClient;
PubSubClient client(espClient);
DHT dht(DHTPIN, DHTTYPE);
Adafruit_TSL2561_Unified tsl = Adafruit_TSL2561_Unified(TSL2561_ADDR_FLOAT, 12345);



void IRAM_ATTR pulseCounter(void);
void liquidQty(byte *p_pulse1Sec, unsigned int *p_flowMilliLitres, unsigned long *p_totalMilliLitres, float *p_flowRate, unsigned long *p_previousMillis);
void conectaWiFi(void);
void conectaMQTT(void);
void callback(char *topic, byte *payload, unsigned int length);
void acionaBomba(bool payload);
void displayInfo(float *p_temp, float *p_umid, float *p_vazao, unsigned long *p_totalMilliLitres, float *p_umidSolo, bool *p_solenoide, float *p_lux, char **p_dados);
void enviaDadosMQTT(char **p_dados);
void pegaValores(float *p_temp, float *p_umid, float *p_umidSolo, float *p_lux);
double conversaoSolo(float valorAdq, float valorMin, float valorMax, float tensaoMin, float tensaoMax);
void verificaBomba(bool *p_solenoide);
void controleCooler(byte *p_pwm);







void setup() {
  pinMode(SENSOR, INPUT_PULLUP);
  pinMode(RELE, OUTPUT);
  pinMode(SOLO_SENSOR, INPUT);
  pinMode(COOLER, OUTPUT);
  Serial.begin(115200);
  Serial.println("Inicio do programa");

  conectaWiFi();
  conectaMQTT();

  dht.begin();

  if(!tsl.begin())
  {
    Serial.print("Ooops, no TSL2561 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }

  attachInterrupt(digitalPinToInterrupt(SENSOR), pulseCounter, FALLING);

}



void loop() {
  unsigned long previousMillis = 0, previousMillis2 = 0;
  byte pulse1Sec = 0;
  unsigned int flowMilliLitres = 0;
  unsigned long totalMilliLitres = 0;
  float flowRate = 0.0, t = 0, h = 0, umidSolo = 0, lux = 0;
  long currentMillis = 0;
  char *p_dados;
  bool solenoide = false;


  while(1)
  {
    pegaValores(&t, &h, &umidSolo, &lux);
    verificaBomba(&solenoide);
    currentMillis = millis();
    if(currentMillis - previousMillis > 1000)
    {
      liquidQty(&pulse1Sec, &flowMilliLitres, &totalMilliLitres, &flowRate, &previousMillis);
      displayInfo(&t, &h, &flowRate, &totalMilliLitres, &umidSolo, &solenoide, &lux, &p_dados);
      enviaDadosMQTT(&p_dados);
      previousMillis = currentMillis;
    }

    if(currentMillis - previousMillis2 > 15000)
    {
      
      previousMillis2 = currentMillis;
    }
    client.loop();
  }
}



void IRAM_ATTR pulseCounter(void)
{
  pulseCount++;
}


void liquidQty(byte *p_pulse1Sec, unsigned int *p_flowMilliLitres, unsigned long *p_totalMilliLitres, float *p_flowRate, unsigned long *p_previousMillis)
{
  float calibrationFactor = 4.5;

  *p_pulse1Sec = pulseCount;
  pulseCount = 0;

  *p_flowRate = ((1000.0 / (millis() - *p_previousMillis)) * *p_pulse1Sec) / calibrationFactor;
  *p_previousMillis = millis();

  *p_flowMilliLitres = (*p_flowRate / 60) * 1000;

  *p_totalMilliLitres += *p_flowMilliLitres;
}

void conectaWiFi(void)
{
  WiFi.begin("WIFI-FACENS", "iOt#F@c0504");

  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi conectado");
  Serial.println("IP obtido: ");
  Serial.println(WiFi.localIP());
}

void conectaMQTT(void)
{
  const char *mqttServer = "52.1.135.87";
  const int mqttPort = 1883;

  client.setServer(mqttServer, mqttPort);
  client.setCallback(callback);

  Serial.println("\nConectando no MQTT...");
  while(!client.connected())
  {
    if(client.connect("Sensor1"))
    {
      Serial.println("Conectado!");
      Serial.println("Conectado Ao servidor MQTT!");
      client.subscribe("smartGreenHouseRead");
      client.subscribe("smartGreenHousePWM");
    }else
    {
      Serial.print("Falha na conexão ao servidor MQTT. Código de erro: ");
      Serial.print(client.state());
      delay(2000);
    }
  }
}

void callback(char *topic, byte *payload, unsigned int length)
{
  String valor = "";
  Serial.print("Mensagem recebida: ");
  for(int i = 0; i < length; i++)
  {
    valor += (char)payload[i];
    Serial.print((char)payload[i]);
  }
  Serial.println("");


  if(strcmp(topic, "smartGreenHouseRead") == 0)
  {
    if(valor == "1")
      acionaBomba(true);
    else if(valor == "0")
      acionaBomba(false);
  }

  if(strcmp(topic, "smartGreenHousePWM") == 0)
  {
    byte pwm = valor.toInt();
    controleCooler(&pwm);
  }
  

  valor = "";
}


void acionaBomba(bool payload)
{
  Serial.print("Recebeu payload: ");
  Serial.println(payload);

  if(payload == true)
  {
    digitalWrite(RELE, HIGH);
    Serial.println("Bomba ligada");
  }else
  {
    digitalWrite(RELE, LOW);
    Serial.println("Bomba desligada");
  }
}


void displayInfo(float *p_temp, float *p_umid, float *p_vazao, unsigned long *p_totalMilliLitres, float *p_umidSolo, bool *p_solenoide, float *p_lux, char **p_dados)
{
  int tamanho_string = snprintf(NULL, 0, "temp: %.2f°C | hum: %.2f%% | flow: %.2f | vol: %iml | soil: %.2f%% | lux: %.2f | solenoide: %c\n", *p_temp, *p_umid, *p_vazao, *p_totalMilliLitres, *p_umidSolo, *p_lux,*p_solenoide ? '1' : '0');
  *p_dados = (char*)malloc((tamanho_string + 1) * sizeof(char));

  if(*p_dados != NULL)
  {
    sprintf(*p_dados, "temp: %.2f°C | hum: %.2f%% | flow: %.2f | vol: %iml | soil: %.2f%% | lux: %.2f | status: %c\n", *p_temp, *p_umid, *p_vazao, *p_totalMilliLitres, *p_umidSolo, *p_lux, *p_solenoide ? '1' : '0');
    Serial.println(*p_dados);
  }else
  {
    Serial.println("Erro ao alocar memoria");
    delay(1000);
    ESP.restart();
  }
}

void enviaDadosMQTT(char **p_dados)
{
  client.publish("smartGreenHouse", *p_dados);
  Serial.println("Publicado");
  free(*p_dados);
}

void pegaValores(float *p_temp, float *p_umid, float *p_umidSolo, float *p_lux)
{
  sensors_event_t event;
  tsl.getEvent(&event);

  if(event.light)
    *p_lux = event.light;
  *p_temp = dht.readTemperature();
  *p_umid = dht.readHumidity();
  *p_umidSolo = conversaoSolo(analogRead(SOLO_SENSOR), 0, 4095, 100, 0);
}

double conversaoSolo(float valorAdq, float valorMin, float valorMax, float tensaoMin, float tensaoMax)
{
  return (valorAdq - valorMin) * (tensaoMax - tensaoMin) / (valorMax - valorMin) + tensaoMin;
}

void verificaBomba(bool *p_solenoide)
{
  if(digitalRead(RELE) == HIGH)
    *p_solenoide = true;
  else
    *p_solenoide = false;
}

void controleCooler(byte *p_pwm)
{
  analogWrite(COOLER, *p_pwm);
}

void controleLED(byte *p_pwm)
{
  analogWrite(LED, *p_pwm);
}
