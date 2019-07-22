//Programa : Arduino Ethernet Shield W5100 e HC-SR04
//Alteracoes e adaptacoes : FILIPEFLOP
//Baseado no programa exemplo de
//by David A. Mellis e Tom Igoe
// baseado nisso e em muitas outras coisas...


#include <DHT_U.h>
#include <DHT.h>
#include <SPI.h>
#include <Ethernet.h>

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>


#include"AirQuality.h"
#include "SI114X.h"



#define MAXDELTA 5000

#define BMP_SCK 13
#define BMP_MISO 12
#define BMP_MOSI 11 
#define BMP_CS 10

//AirQuality airqualitysensor;
Adafruit_BMP280 sensor_bmp;
// Comunicacao via SPI:
/////Adafruit_BMP280 sensor_bmp(BMP_CS, BMP_MOSI, BMP_MISO,  BMP_SCK);
/////Adafruit_BMP280 sensor_bmp(BMP_CS);

SI114X uv = SI114X();



int current_quality =-1;
int smokeA0 = A3;
int sensorThres = 900;
unsigned long timeStart = 0;
float t,h;
float temp2 = 0, pressure = 0, alt = 0;
int analogSensor;
unsigned long logtime = 0;
float uvVis;
float uvIR;
float uvUV;


// Define pino e tipo do sensor DHT
DHT dht(3, DHT22);
  
//Definicoes de IP, mascara de rede e gateway
byte mac[] = {
  0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };
IPAddress ip(192,168,10,69);          //Define o endereco IP
IPAddress gateway(192,168,10,1);     //Define o gateway
IPAddress subnet(255, 255, 255, 0); //Define a máscara de rede
 
//Inicializa o servidor web na porta 80
EthernetServer server(80);
 
void setup()
{
  Serial.begin(9600);
  Serial.println("Funcao setup");
    pinMode(smokeA0, INPUT);
  //Inicializa a interface de rede
  Ethernet.begin(mac, ip, gateway, subnet);
  server.begin();
  dht.begin();
  
  if (!sensor_bmp.begin()) {  
    Serial.println("Could not find a valid BMP280 sensor, check wiring!");
    while (1);
  }
  ////Serial.write("Valor inicializacao: ");
  ///Serial.println(ret);

 /* Default settings from datasheet. */
 // sensor_bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
 //                 Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
 //                 Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
 //                 Adafruit_BMP280::FILTER_X16,      /* Filtering. */
 //                 Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */


 while(!uv.Begin()) {
    Serial.println("Didn't find Si1145");
    delay(1000);
  }

    Serial.println("Setup OK!");
  
  
}


void sensorMeasurement()
{
  unsigned long deltat = millis() - timeStart;
  logtime = deltat;
  if( deltat >= MAXDELTA )
  {
    timeStart = millis();
    t = dht.readTemperature();
    h = dht.readHumidity();
    //current_quality=airqualitysensor.slope();
    analogSensor = analogRead(smokeA0);
    temp2 = sensor_bmp.readTemperature();
    pressure = sensor_bmp.readPressure();
    Serial.print(pressure);
    // TODO: conferir valor do altitude!
    alt = sensor_bmp.readAltitude(1013.25);

    uvVis = uv.ReadVisible();
    uvIR = uv.ReadIR();
    uvUV = uv.ReadUV();
    
  }
}
 
void loop() {

  
 
  //Aguarda conexao do browser
  EthernetClient client = server.available();
  if (client) {
    Serial.println("new client");
    // an http request ends with a blank line
    boolean currentLineIsBlank = true;
    while (client.connected()) {
      if (client.available()) {
        char c = client.read();
        Serial.write(c);
        // if you've gotten to the end of the line (received a newline
        // character) and the line is blank, the http request has ended,
        // so you can send a reply
        if (c == 'n' && currentLineIsBlank) {
          // send a standard http response header
          client.println("HTTP/1.1 200 OK");
          client.println("Content-Type: text/html");
          client.println("Connection: close");
          client.println("Refresh: 2"); //Recarrega a pagina a cada 2seg
          client.println();
          client.println("<!DOCTYPE HTML>");
          client.println("<html>");
          //Configura o texto e imprime o titulo no browser
          client.print("<font color=#FF0000><b><u>");
          client.print("Weather Station v0.4 / 2019-07-16:");
          client.print("</u></b></font>");
          client.println("<br />");
          client.println("<br />");
          //Mostra o estado da porta digital 3
          //int porta_digital = digitalRead(3);
          sensorMeasurement();
          client.print("DATA temperatura : ");
          //// client.print("<b>");
    
          //Mostra as informacoes lidas pelo sensor ultrasonico
  // Leitura temperatura, umidade e pressao
////  float t = dht.readTemperature();

        client.print(t);
          client.println(" oC </b>");
 client.println("<br />");

         client.print("DATA umidade : ");
  ///        client.print(" <b>");
    
          //Mostra as informacoes lidas pelo sensor ultrasonico

  // Leitura temperatura, umidade e pressao
////  float h = dht.readHumidity();

        client.print(h);
          client.println(" %</b>");
 client.println("<br />");



///  int analogSensor = analogRead(smokeA0);
 // Checks if it has reached the threshold value
  if (analogSensor > sensorThres)
  {
    client.println("FIRE!!!!!");
  }
  else
  {
    client.println("no fire, everything ok...");
  }
  client.println("<br />");
  client.println("Logtime: ");
  client.println(logtime);
  client.println("/time now:");
  client.println(millis());

          
          client.println("<br />");

        client.print("DATA Temperatura 2: ");
        client.print(temp2);
        client.println(" oC </b>");
        client.println("<br />");


        client.print("DATA Pressao: ");
        client.print(pressure);
        client.println(" Pa </b>");
        client.println("<br />");


        client.print("DATA Altitude:  ");
        client.print(alt);
        client.println(" m </b>");
        client.println("<br />");


        client.print("DATA Visible light:  ");
        client.print(uvVis);
        client.println("<br />");

        client.print("DATA IR light:  ");
        client.print(uvIR);
        client.println("<br />");

        client.print("DATA UV light:  ");
        client.print(uvUV);
        client.println("<br />");

        client.print("DATA UV index:  ");
        float uvindex = uvUV/100.0;
        client.print(uvindex);
        client.println("<br />");


        client.print("DATA MQ2 GAS Sensor:  ");
        client.print(analogSensor);
        client.println("<br />");

          
          client.println("</b></html>");
          break;
        }
        if (c == 'n') {
          // you're starting a new line
          currentLineIsBlank = true;
        } 
        else if (c != 'r') {
          // you've gotten a character on the current line
          currentLineIsBlank = false;
        }
      }
    }
    // give the web browser time to receive the data
    delay(1);
    // close the connection:
    client.stop();
    }
}
