#include <FS.h>
//#include <Arduino.h>
#include "PCF8574.h"
#include <Adafruit_Sensor.h> // Adafruit Unified Sensor
#include <Adafruit_BMP280.h> // modificar arquivo cpp - 0x77 . 0x76
#include "Adafruit_Si7021.h"
#include <ArduinoJson.h> // Versão 5.13
#include <PubSubClient.h>
#include <ESP8266WiFi.h>
#include <ESP8266WiFiMulti.h>
#include <ArduinoOTA.h>


#define OTHER_PIN               D7    // Pino digital para controlar o clock do CI CD4014
#define INTERRUPTED_PIN         D8    // Interrupt Pin for PCF8574 in the WEMOS board
#define ANEMOMETRO_PIN          D6    // Pino digital para receber os dados do Anemômetro
#define PLUVIOMETRO_PIN         D5    // Pino digital para receber os dados do Pluviômetro


#define ENDERECO_I2C_PCF8574    0x20  // Endereço I2C do SMD PCF8574

// -----------------------------------------------------------------

const float pi = 3.14159265;                      // Número de pi
const int periodoAfericaoVelocidadeVento = 30000; // Período Aferição Vento entre as medidas (ms)
const int raioAnemometro = 65;                    // Raio do anemometro(mm)

extern float velocidadeVento;                        // Velocidade do vento (km/h)
extern boolean velocidadeDoVentoAferida;

unsigned long dataUltimoEnvioDadosServidor;
const int periodoAfericaoGeral = 5000; // Período Aferição Geral entre as medidas (ms)

// -----------------------------------------------------------------

volatile byte numPulsosAnemometro = 0;                          // Contador para o sensorSi7021reed switch no anemometro
unsigned int  rpm = 0;                                          // Rotações por minuto do anemometro
float         velocidadeVento = 0;                              // Velocidade do vento (km/h)
unsigned long dataUltimoAfericaoVelocidadeVento = millis();
boolean velocidadeDoVentoAferida = false;

// -----------------------------------------------------------------#

float voltagem = 0.0;
unsigned int raw = 0;

// -----------------------------------------------------------------#
void tratarInterrupcaoPCF8574();

// Set i2c HEX address
PCF8574 pcf8574(ENDERECO_I2C_PCF8574, D2, D1, INTERRUPTED_PIN, tratarInterrupcaoPCF8574);
bool keyPressed = false;

// Variáveis auxiliáres utilizadas para definir a direção do Vento.
byte direct = 1; //Inicia com a posição Norte
int pointer = 0;
const String compass[] = {"N  ", "NNE", "NE ", "NEE",
                          "E  ", "SEE", "SE ", "SSE",
                          "S  ", "SSW", "SW ", "SWW",
                          "W  ", "NWW", "NW ", "NNW",
                          "???"
                         };
PCF8574::DigitalInput valores;

// -----------------------------------------------------------------#

const int periodoAfericaoPluviometria = 5000;   // Período Aferição Pluviometro
const float mLBatidasBascula = 10.0;            // Quantidade de mL por batida da bascula (mL)
const float areaPluviometro = 56.72;            // Area do pluviometro (cm²)

volatile byte numPulsosPluviometro = 0; // Contador para o sensor reed switch no pluviometro
float quantidadeChuva = 0;
unsigned long dataUltimaAfericaoPluviometria = millis();

boolean pluviometriaAferida = false;

// -----------------------------------------------------------------#

float altitude; // this should be adjusted to your local forcase
float temperaturaBMP280;
float pressaoAtmosferica;
float umidadeAr;
float temperaturaSI7021;

Adafruit_BMP280 sensorBMP; // I2C - Lembrar de ajustar o BMP280_ADDRESS para 0x76 em Adafruit_BMP280.h
Adafruit_Si7021 sensorSi7021 = Adafruit_Si7021();

// -----------------------------------------------------------------#
#define WIFISSID01 "extensao_iot" // Put your WifiSSID here
#define PASSWORD01 "aluno123" // Put your wifi password here

#define WIFISSID02 "IFPE-PUBLICA" // Put your WifiSSID here
#define PASSWORD02 "aluno123" // Put your wifi password here

#define WIFISSID03 "extensao_iot" // Put your WifiSSID here
#define PASSWORD03 "aluno123" // Put your wifi password here

char servidorMqtt[] = "172.26.1.8";
char portaServidorMqtt[6] = "1883";
char tokenMqttDisp[33] = "ESP8266_TKN_WEMOS";

ESP8266WiFiMulti wifiMulti;
WiFiClient wifiClient;
PubSubClient client(wifiClient);

bool otaTime = false;

// -----------------------------------------------------------------#


void setup()
{
  wifi_set_sleep_type(MODEM_SLEEP_T);
  // Inicializa as variáveis globais
  dataUltimoEnvioDadosServidor = millis();
  Serial.begin(115200); // Define e inicializa a porta para debugar
  Serial.println();
  Serial.println("Starting setup...");
  pinMode(A0, INPUT);                                                                                // Definição do Pino analógico para aferição da carga da bateria
  pinMode(ANEMOMETRO_PIN, INPUT_PULLUP);                                                             // habilita o resistor interno de pull_up. Iniciando a saida do pino 2 como HIGH (1).
  pinMode(PLUVIOMETRO_PIN, INPUT_PULLUP);                                                            // habilita o resistor interno de pull_up. Iniciando a saida do pino 2 como HIGH (1).
  pinMode(OTHER_PIN, OUTPUT);                                                                       // habilita o resistor interno de pull_up. Iniciando a saida do pino 2 como HIGH (1).
  digitalWrite(OTHER_PIN, HIGH);
  delay(5000); //energizar o pcf8574
  attachInterrupt(digitalPinToInterrupt(ANEMOMETRO_PIN), contarQtdInterrupcoesAnemometro, RISING);   // interrupção 0 está ligado ao pino 2 do arduino. Falling = HIGH > LOW.
  attachInterrupt(digitalPinToInterrupt(PLUVIOMETRO_PIN), contarQtdInterrupcoesPluviometro, FALLING); // interrupção 0 está ligado ao pino 2 do arduino. Falling = HIGH > LOW.
  wifiScan();
  beginClient();
  inicializarSensores();
  // Set output mode for all GPIO pins
  // Define pin modes
  inicializarPCF8574();
  imprimirInformacoesWifiEMQTT();  
}

void imprimirValoresSensoreamento();
void enviarInformacoesParaServidorMQTT();
void enviarInfoVelocidadeVentoServidor();
void enviarInfoPluviometriaParaServidor();
void aferirEEnviarInformacoesSensores();

void loop()
{
  ArduinoOTA.handle();
  bool isConnected = connected() ? true : conectar();
  if (isConnected) {    
    //loopClient();
    aferirVelocidadeDoVento();   
    aferirPluviometria();    
    aferirEEnviarInformacoesSensores();
    imprimirValoresSensoreamento();     
    if(velocidadeDoVentoAferida && pluviometriaAferida){
      enviarInfoVelocidadeVentoServidor();
      imprimirInformacoesVelVento();
      resetarContadoresAnemometro();
      
      enviarInfoPluviometriaParaServidor();
      imprimirValoresPluviometro();
      resetarContadoresPluviometro();     

      finalizarPCF8574();
      
      Serial.println("Tempo reservado para o OTA!");    
      setupOTA();
      //delay(3*60*1000);
      Serial.println("Hora de dormir!");
      ESP.deepSleep(1e7);  
    }
    delay(1000);
  } else {
    Serial.println("Nao conectado.");
    delay(1000);
  }
}

void setupOTA(){
  // Port defaults to 8266
  // ArduinoOTA.setPort(8266);

  // Hostname defaults to esp8266-[ChipID]
  // ArduinoOTA.setHostname("myesp8266");

  // No authentication by default
  // ArduinoOTA.setPassword("admin");

  // Password can be set with it's md5 value as well
  // MD5(admin) = 21232f297a57a5a743894a0e4a801fc3
  // ArduinoOTA.setPasswordHash("21232f297a57a5a743894a0e4a801fc3");

  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH) {
      type = "sketch";
    } else { // U_SPIFFS
      type = "filesystem";
    }

    // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
    Serial.println("Start updating " + type);
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) {
      Serial.println("Auth Failed");
    } else if (error == OTA_BEGIN_ERROR) {
      Serial.println("Begin Failed");
    } else if (error == OTA_CONNECT_ERROR) {
      Serial.println("Connect Failed");
    } else if (error == OTA_RECEIVE_ERROR) {
      Serial.println("Receive Failed");
    } else if (error == OTA_END_ERROR) {
      Serial.println("End Failed");
    }
  });
  ArduinoOTA.begin();
  Serial.println("Ready");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

void aferirEEnviarInformacoesSensores() {
  //if (millis() > dataUltimoEnvioDadosServidor + periodoAfericaoGeral)
  //{
    Serial.println("Inicia Leitura...");
    aferirCargaBateria();
    definirDirecaoDoVento();
    lerValorSensores();
    enviarInformacoesParaServidorMQTT();    
    dataUltimoEnvioDadosServidor = millis();
  //}
}

void enviarInfoVelocidadeVentoServidor() {
  if (velocidadeDoVentoAferida) {
    String velocidadeVentoStr = String(velocidadeVento);
    enviarInfoParaServidorMQTT("windSpeed", velocidadeVentoStr);
    velocidadeDoVentoAferida = false;
  }
}

void enviarInfoPluviometriaParaServidor() {
  if (pluviometriaAferida) {
    String quantidadeChuvaStr = String(quantidadeChuva);
    enviarInfoParaServidorMQTT("quantidadeChuva", quantidadeChuvaStr);
    pluviometriaAferida = false;
  }
}

void imprimirValoresSensoreamento()
{  
  imprimirDirecaoVentoAferida();
  imprimirValoresBateria();
  imprimirValorSensores();
  Serial.println("-------------------------------");
}

void enviarInformacoesParaServidorMQTT()
{
  String altitudeStr = String(altitude);
  String temperatureBMP280Str = String(temperaturaBMP280);
  String pressureStr = String(pressaoAtmosferica);
  String humidityStr = String(umidadeAr);
  String temperatureSI7021Str = String(temperaturaSI7021);
  String voltStr = String(voltagem);
  String pointerStr = String(pointer);

  enviarInfoParaServidorMQTT("temperatureBMP280", temperatureBMP280Str);
  enviarInfoParaServidorMQTT("pressure", pressureStr);
  enviarInfoParaServidorMQTT("altitude", altitudeStr);
  enviarInfoParaServidorMQTT("humidity", humidityStr);
  enviarInfoParaServidorMQTT("battery", voltStr);
  enviarInfoParaServidorMQTT("compass", pointerStr);
  enviarInfoParaServidorMQTT("temperatureSI7021", temperatureSI7021Str);
}

// -----------------------------------------------------------------

//Função para calcular o RPM
void calcularRpmAnemometro()
{
  rpm = ((numPulsosAnemometro) * 60) / (periodoAfericaoVelocidadeVento / 1000);
}

//Função para calcular a velocidade do vento em km/h
void calcularVelocidadeVento()
{
  velocidadeVento = (((2 * pi * raioAnemometro * rpm) / 60) / 1000) * 3.6;
}

// --- Funções de callback de interrupção ---

//Função para contar o número de vezes que o sensor reed switch fechou (pulsos do anemometro)
ICACHE_RAM_ATTR void contarQtdInterrupcoesAnemometro()
{
  static unsigned long tempoUltimaInterrupcaoAnem = 0;
  unsigned long tempoInterrupcao = millis();
  if (tempoInterrupcao - tempoUltimaInterrupcaoAnem > 200)
  { //faz o debounce do reed switch
    numPulsosAnemometro++;
  }
}

//Função para resetar os contadores
void resetarContadoresAnemometro() {
  numPulsosAnemometro = 0;
}

void imprimirInformacoesVelVento()
{
  Serial.print("Pulsos anemômetro: ");
  Serial.print(numPulsosAnemometro);
  Serial.print(";  RPM: ");
  Serial.print(rpm);
  Serial.print(";  Vel. Vento: ");
  Serial.print(velocidadeVento);
  Serial.print(" [km/h] ");
  Serial.println();
}

void aferirVelocidadeDoVento()
{
  if (millis() > dataUltimoAfericaoVelocidadeVento + periodoAfericaoVelocidadeVento)
  {
    calcularRpmAnemometro();
    calcularVelocidadeVento();    
    dataUltimoAfericaoVelocidadeVento = millis();
    velocidadeDoVentoAferida = true;
  }
}

// -----------------------------------------------------------------

void aferirCargaBateria()
{
  raw = analogRead(A0);
  float volt = map(raw, 665, 980, 280, 410);
  voltagem = volt / 100.0;
}

void imprimirValoresBateria()
{
  Serial.print("Carga Bateria: Raw = ");
  Serial.print(raw);
  Serial.print("; Voltage = ");
  Serial.print(voltagem);
  Serial.print(" [V]");
  Serial.println();
}


// -----------------------------------------------------------------


void inicializarPCF8574()
{
  pcf8574.pinMode(P0, INPUT);
  pcf8574.pinMode(P1, INPUT);
  pcf8574.pinMode(P2, INPUT);
  pcf8574.pinMode(P3, INPUT);
  pcf8574.pinMode(P4, INPUT);
  pcf8574.pinMode(P5, INPUT);
  pcf8574.pinMode(P6, INPUT);
  pcf8574.pinMode(P7, INPUT);
  pcf8574.begin();
}

void finalizarPCF8574()
{
  digitalWrite(OTHER_PIN, LOW);
}

byte lerPinosDirecaoVento()
{
  valores = pcf8574.digitalReadAll();
  byte valorPinos = 0;
  valorPinos = valorPinos | (valores.p0 << 7);
  valorPinos = valorPinos | (valores.p1 << 6);
  valorPinos = valorPinos | (valores.p2 << 5);
  valorPinos = valorPinos | (valores.p3 << 4);
  valorPinos = valorPinos | (valores.p4 << 3);
  valorPinos = valorPinos | (valores.p5 << 2);
  valorPinos = valorPinos | (valores.p6 << 1);
  valorPinos = valorPinos | (valores.p7 << 0);
  return valorPinos;
}

void definirDirecaoDoVento()
{
  pointer = -1;
  direct = lerPinosDirecaoVento();  
  switch (direct)
  {
    case 1:
      pointer = 0;
      break;
    case 3:
      pointer = 1;
      break;
    case 2:
      pointer = 2;
      break;
    case 6:
      pointer = 3;
      break;
    case 4:
      pointer = 4;
      break;
    case 12:
      pointer = 5;
      break;
    case 8:
      pointer = 6;
      break;
    case 24:
      pointer = 7;
      break;
    case 16:
      pointer = 8;
      break;
    case 48:
      pointer = 9;
      break;
    case 32:
      pointer = 10;
      break;
    case 96:
      pointer = 11;
      break;
    case 64:
      pointer = 12;
      break;
    case 192:
      pointer = 13;
      break;
    case 128:
      pointer = 14;
      break;
    case 129:
      pointer = 15;
      break;
    default:
      pointer = 16;
      // if nothing else matches, do the default
      // default 16, "???" mainly for debugging
      break;
  }
}

ICACHE_RAM_ATTR void tratarInterrupcaoPCF8574()
{
  // Interrupt called (No Serial no read no wire in this function, and DEBUG disabled on PCF library)
  static unsigned long tempoUltimaInterrupcaoCompass = 0;
  unsigned long tempoInterrupcao = millis();
  if (tempoInterrupcao - tempoUltimaInterrupcaoCompass > 200)
  { //faz o debounce do reed switch
    definirDirecaoDoVento();
  }
}

void imprimirDirecaoVentoAferida()
{
  Serial.print("Direção do Vento: ");
  Serial.print(direct);
  Serial.print("\t . ");
  Serial.print(pointer);
  Serial.print("\t . ");
  Serial.print(compass[pointer]);
  //Serial.println(direct, BIN);
  Serial.println();
}

void imprimirPinosDirecaoVento()
{
  Serial.print("Pinos do PCF8574:");
  Serial.print(valores.p0);
  Serial.print(valores.p1);
  Serial.print(valores.p2);
  Serial.print(valores.p3);
  Serial.print(valores.p4);
  Serial.print(valores.p5);
  Serial.print(valores.p6);
  Serial.print(valores.p7);
  Serial.println();
}

// -----------------------------------------------------------------

//Função para resetar os contadores
void resetarContadoresPluviometro()
{
  numPulsosPluviometro = 0;
  unsigned long millis();
}

//Função para calcular a quantidade de chuva.
void calcularQuantidadeChuva()
{
  quantidadeChuva = ((numPulsosPluviometro * mLBatidasBascula) / areaPluviometro) * 10;
}

// --- Funções de callback de interrupção ---

//Função para contar o número de vezes que o sensor reed switch fechou (pulsos do pluviomometro)
ICACHE_RAM_ATTR void contarQtdInterrupcoesPluviometro()
{
  static unsigned long tempoUltimaInterrupcaoPluv = 0;
  unsigned long tempoInterrupcao = millis();
  if (tempoInterrupcao - tempoUltimaInterrupcaoPluv > 200) //faz o debounce do reed switch
  {
    numPulsosPluviometro++;
  }
}

void imprimirValoresPluviometro()
{
  Serial.print("Pulsos pluviômetro: ");
  Serial.print(numPulsosPluviometro);
  Serial.print("; Qtd. Chuva: ");
  Serial.print(quantidadeChuva);
  Serial.print(" [m3] ");
  Serial.println();
}

void aferirPluviometria()
{
  if (millis() > dataUltimaAfericaoPluviometria + periodoAfericaoPluviometria)
  {
    calcularQuantidadeChuva();   
    dataUltimaAfericaoPluviometria = millis();
    pluviometriaAferida = true;
  }
}


// -----------------------------------------------------------------

void inicializarSensores()
{
  if (!sensorBMP.begin())
  {
    Serial.println("Could not find a valid BMP280 sensor, check wiring!");
    while (true)
      ;
  }
  if (!sensorSi7021.begin())
  {
    Serial.println("Did not find Si7021 sensor!");
    while (true)
      ;
  }
}

void lerValorSensores()
{
  altitude = sensorBMP.readAltitude(1013.25); // this should be adjusted to your local forcase
  temperaturaBMP280 = sensorBMP.readTemperature();
  pressaoAtmosferica = sensorBMP.readPressure();
  umidadeAr = sensorSi7021.readHumidity();
  temperaturaSI7021 = sensorSi7021.readTemperature();
}

void imprimirValorSensores()
{
  Serial.println("BMP280 results");
  Serial.print("Temperature = ");
  Serial.print(temperaturaBMP280);
  Serial.println(" *C");
  Serial.print("Pressure = ");
  Serial.print(pressaoAtmosferica);
  Serial.println(" Pa");
  Serial.print("Approx altitude = ");
  Serial.print(altitude);
  Serial.println(" m");
  Serial.println();
  Serial.println("SI7021 results");
  Serial.print("Humidity: ");
  Serial.println(umidadeAr, 2);
  Serial.print("Temperature: ");
  Serial.println(temperaturaSI7021, 2);
  Serial.println();
}

// -----------------------------------------------------------------

boolean connected() {
  return client.connected();
}

boolean loopClient() {
  if (connected()) {
    return client.loop();
  }
  return false;
}

void carregarConfigsDoSistemaDeArquivos() {
  delay(10000);
  Serial.println("mounting FS...");
  if (SPIFFS.begin()) {
    Serial.println("mounted file system");
    if (SPIFFS.exists("/config.json")) {
      //file exists, reading and loading
      Serial.println("reading config file");
      File configFile = SPIFFS.open("/config.json", "r");
      if (configFile) {
        Serial.println("opened config file");
        size_t size = configFile.size();
        // Allocate a buffer to store contents of the file.
        std::unique_ptr<char[]> buf(new char[size]);
        configFile.readBytes(buf.get(), size);
        DynamicJsonBuffer jsonBuffer;
        JsonObject& json = jsonBuffer.parseObject(buf.get());
        json.printTo(Serial);
        if (json.success()) {
          Serial.println("\nparsed json");
          if (json["servidorMqtt"]) {
            strcpy(servidorMqtt , json["servidorMqtt"]);
            Serial.print("Servidor MQTT copiado!  ");
            Serial.println(servidorMqtt);
          }
          if (json["portaServidorMqtt"]) {
            strcpy(portaServidorMqtt, json["portaServidorMqtt"]);
            Serial.println("Porta MQTT copiada!");
          }
          if (json["tokenMqttDisp"]) {
            strcpy(tokenMqttDisp, json["tokenMqttDisp"]);
            Serial.println("Token copiado!");
          }
          if (json["ip"]) {
            Serial.println("setting custom ip from config");
            //static_ip = json["ip"];
            //strcpy(static_ip, json["ip"]);
            //strcpy(static_gw, json["gateway"]);
            //strcpy(static_sn, json["subnet"]);
            //Serial.println(static_ip);
          } else {
            Serial.println("no custom ip in config");
          }
        } else {
          Serial.println("failed to load json config");
        }
      } else {
        Serial.println("failed to open json config!");
      }
    } else {
      Serial.println("Config.json não existe!");
    }
  } else {
    Serial.println("failed to mount FS");
  }
}

bool inicializarWiFi() {
  bool isConnected = false;
  WiFi.mode(WIFI_STA);
  wifiMulti.addAP(WIFISSID01, PASSWORD01);
  wifiMulti.addAP(WIFISSID02, PASSWORD02);
  wifiMulti.addAP(WIFISSID03, PASSWORD03);
  Serial.println("Connecting Wifi...");
  if (wifiMulti.run() == WL_CONNECTED) {
    Serial.println("");
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
    isConnected = true;
  } else {
    Serial.println("WiFi is not connected!");
  }
  return isConnected;
}

void salvarConfigsNoSistemaDeArquivos() {
  Serial.println("saving config");
  DynamicJsonBuffer jsonBuffer;
  JsonObject& json = jsonBuffer.createObject();
  json["servidorMqtt"] = servidorMqtt;
  json["portaServidorMqtt"] = portaServidorMqtt;
  json["tokenMqttDisp"] = tokenMqttDisp;
  json["ip"] = WiFi.localIP().toString();
  json["gateway"] = WiFi.gatewayIP().toString();
  json["subnet"] = WiFi.subnetMask().toString();
  File configFile = SPIFFS.open("/config.json", "w");
  if (!configFile) {
    Serial.println("failed to open config file for writing");
  }
  json.prettyPrintTo(Serial);
  json.printTo(configFile);
  configFile.close();
  SPIFFS.end();
  //end save
}

bool conectar() {
  // Loop until we're reconnected
  Serial.println("Reconectando...");
  //status = WiFi.status();
  Serial.print("Wifi status: ");
  Serial.println(WiFi.status());
  Serial.println(WiFi.SSID());
  bool isWifiConnected = WiFi.status() != WL_CONNECTED ? true : inicializarWiFi();
  bool isMQTTConnected = false;
  if (isWifiConnected) {
    Serial.print("Connecting to ThingsBoard node ...");
    // Attempt to connect (clientId, username, password)
    isMQTTConnected = client.connect("ESP8266 Device", tokenMqttDisp, NULL);
    if ( isMQTTConnected ) {
      Serial.println( "[DONE]" );
      // Subscribing to receive RPC requests
      client.subscribe("v1/devices/me/rpc/request/+");
      // Sending current GPIO status
      Serial.println("Sending current GPIO status ...");
      //client.publish("v1/devices/me/attributes", get_gpio_status().c_str());
    } else {
      Serial.print( "[FAILED] [ rc = " );
      Serial.print( client.state() );
      Serial.println( " : retrying in 5 seconds]" );
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
  return isMQTTConnected;
}

void beginClient() {
  //carregarConfigsDoSistemaDeArquivos();
  Serial.println("Starting wifi...");
  inicializarWiFi();
  //salvarConfigsNoSistemaDeArquivos();
  client.setServer(servidorMqtt , 1883);
}


void imprimirInformacoesWifiEMQTT() {
  // Imprimir os valores-padrão das variáveis referentes ao WiFi e servidor MQTT
  Serial.println(tokenMqttDisp);
  Serial.println(servidorMqtt);
  Serial.println("local ip");
  Serial.println(WiFi.localIP());
  Serial.println(WiFi.gatewayIP());
  Serial.println(WiFi.subnetMask());
}


void enviarInfoParaServidorMQTT(String atributo, String valor) {
  // Prepare a JSON payload string
  String payload = "{";
  payload += "\"" + atributo + "\":";
  payload += valor;
  payload += "}";

  // Send payload
  char attributes[120];
  payload.toCharArray( attributes, 120 );
  client.publish( "v1/devices/me/telemetry", attributes );
}

void wifiScan() {
  int n = WiFi.scanNetworks();
  Serial.println("scan done");
  if (n == 0) {
    Serial.println("no networks found");
  } else {
    Serial.print(n);
    Serial.println(" networks found");
    for (int i = 0; i < n; ++i) {
      // Print SSID and RSSI for each network found
      Serial.print(i + 1);
      Serial.print(": ");
      Serial.print(WiFi.SSID(i));
      Serial.print(" (");
      Serial.print(WiFi.RSSI(i));
      Serial.print(")");
      Serial.println((WiFi.encryptionType(i) == ENC_TYPE_NONE) ? " " : "*");
      delay(10);
    }
  }
  Serial.println("");
}

// -----------------------------------------------------------------

// -----------------------------------------------------------------

// -----------------------------------------------------------------

// -----------------------------------------------------------------
