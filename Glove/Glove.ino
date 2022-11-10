#include <Arduino.h>
#include <Wire.h> // biblioteca de comunicação I2C

#include <ESP8266WiFi.h>
#include <ESP8266WiFiMulti.h>

ESP8266WiFiMulti WiFiMulti;
WiFiClient client;
const String host = "192.168.4.1";
const int httpPort = 80;

/*
 * Definições de alguns endereços mais comuns do MPU6050
 * os registros podem ser facilmente encontrados no mapa de registros do MPU6050
 */
const int MPU_ADDR = 0x68;     // definição do endereço do sensor MPU6050 (0x68)
const int WHO_AM_I = 0x75;     // registro de identificação do dispositivo
const int PWR_MGMT_1 = 0x6B;   // registro de configuração do gerenciamento de  energia
const int GYRO_CONFIG = 0x1B;  // registro de configuração do giroscópio
const int ACCEL_CONFIG = 0x1C; // registro de configuração do acelerômetro
const int ACCEL_XOUT = 0x3B;   // registro de leitura do eixo X do acelerômetro

const int sda_pin = 0; // definição do pino I2C SDA
const int scl_pin = 2; // definição do pino I2C SCL

bool led_state = false;

// variáveis para armazenar os dados "crus" do acelerômetro
int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;

/*
 * Configura a I2C com os pinos desejados 
 * sda_pin -> D5
 * scl_pin -> D6
 */
void initI2C()
{
  Wire.begin(sda_pin, scl_pin);
}

/*
 * Escreve um dado valor em um dado registro
 */
void writeRegMPU(int reg, int val) //aceita um registro e um valor como parâmetro
{
  Wire.beginTransmission(MPU_ADDR); // inicia comunicação com endereço do MPU6050
  Wire.write(reg);                  // envia o registro com o qual se deseja trabalhar
  Wire.write(val);                  // escreve o valor no registro
  Wire.endTransmission(true);       // termina a transmissão
}

/*
 * Lê de um dado registro
 */
uint8_t readRegMPU(uint8_t reg) // aceita um registro como parâmetro
{
  uint8_t data;
  Wire.beginTransmission(MPU_ADDR); // inicia comunicação com endereço do MPU6050
  Wire.write(reg);                  // envia o registro com o qual se deseja trabalhar
  Wire.endTransmission(false);      // termina transmissão mas continua com I2C aberto (envia STOP e START)
  Wire.requestFrom(MPU_ADDR, 1);    // configura para receber 1 byte do registro escolhido acima
  data = Wire.read();               // lê o byte e guarda em 'data'
  return data;                      //retorna 'data'
}

/*
 * Procura pelo sensor no endereço 0x68
 */
void findMPU(int mpu_addr)
{
  Wire.beginTransmission(MPU_ADDR);
  int data = Wire.endTransmission(true);

  if (data == 0)
  {
    Serial.print("Dispositivo encontrado no endereço: 0x");
    Serial.println(MPU_ADDR, HEX);
  }
  else
  {
    Serial.println("Dispositivo não encontrado!");
  }
}

/*
 * Verifica se o sensor responde e se está ativo
 */
void checkMPU(int mpu_addr)
{
  findMPU(MPU_ADDR);

  int data = readRegMPU(WHO_AM_I); // Register 117 – Who Am I - 0x75

  if (data == 104)
  {
    Serial.println("MPU6050 Dispositivo respondeu OK! (104)");

    data = readRegMPU(PWR_MGMT_1); // Register 107 – Power Management 1-0x6B

    if (data == 64)
      Serial.println("MPU6050 em modo SLEEP! (64)");
    else
      Serial.println("MPU6050 em modo ACTIVE!");
  }
  else
    Serial.println("Verifique dispositivo - MPU6050 NÃO disponível!");
}

/* 
 * Configurar o sleep bit  
 */
void setSleepOff()
{
  writeRegMPU(PWR_MGMT_1, 0); // escreve 0 no registro de gerenciamento de energia(0x68), colocando o sensor em o modo ACTIVE
}

/* Configurar as escalas do giroscópio
   registro da escala do giroscópio: 0x1B[4:3]
   0 é 250°/s
 
    FS_SEL  Full Scale Range
      0        ± 250 °/s      0b00000000
      1        ± 500 °/s      0b00001000
      2        ± 1000 °/s     0b00010000
      3        ± 2000 °/s     0b00011000
*/
void setGyroScale()
{
  writeRegMPU(GYRO_CONFIG, 0);
}

/* Configurar as escalas do acelerômetro
   registro da escala do acelerômetro: 0x1C[4:3]
   0 é 250°/s
 
    AFS_SEL   Full Scale Range
      0           ± 2g            0b00000000
      1           ± 4g            0b00001000
      2           ± 8g            0b00010000
      3           ± 16g           0b00011000
*/
void setAccelScale()
{
  writeRegMPU(ACCEL_CONFIG, 0);
}

/* Lê os dados 'crus'(raw data) do sensor
   são 14 bytes no total sendo eles 2 bytes para cada eixo e 2 bytes para temperatura:
 
  0x3B 59 ACCEL_XOUT[15:8]
  0x3C 60 ACCEL_XOUT[7:0]
  0x3D 61 ACCEL_YOUT[15:8]
  0x3E 62 ACCEL_YOUT[7:0]
  0x3F 63 ACCEL_ZOUT[15:8]
  0x40 64 ACCEL_ZOUT[7:0]
 
  0x41 65 TEMP_OUT[15:8]
  0x42 66 TEMP_OUT[7:0]
 
  0x43 67 GYRO_XOUT[15:8]
  0x44 68 GYRO_XOUT[7:0]
  0x45 69 GYRO_YOUT[15:8]
  0x46 70 GYRO_YOUT[7:0]
  0x47 71 GYRO_ZOUT[15:8]
  0x48 72 GYRO_ZOUT[7:0]
    
*/
void readRawMPU()
{
  Wire.beginTransmission(MPU_ADDR); // inicia comunicação com endereço do MPU6050
  Wire.write(ACCEL_XOUT);           // envia o registro com o qual se deseja trabalhar, começando com registro 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);      // termina transmissão mas continua com I2C aberto (envia STOP e START)
  Wire.requestFrom(MPU_ADDR, 14);   // configura para receber 14 bytes começando do registro escolhido acima (0x3B)

  AcX = Wire.read() << 8; // lê primeiro o byte mais significativo
  AcX |= Wire.read();     // depois lê o bit menos significativo
  AcY = Wire.read() << 8;
  AcY |= Wire.read();
  AcZ = Wire.read() << 8;
  AcZ |= Wire.read();

  Tmp = Wire.read() << 8;
  Tmp |= Wire.read();

  GyX = Wire.read() << 8;
  GyX |= Wire.read();
  GyY = Wire.read() << 8;
  GyY |= Wire.read();
  GyZ = Wire.read() << 8;
  GyZ |= Wire.read();

  // Serial.print("AcX = ");
  // Serial.print(AcX);
  // Serial.print(" | AcY = ");
  // Serial.print(AcY);
  // Serial.print(" | AcZ = ");
  // Serial.print(AcZ);
  // Serial.print(" | Tmp = ");
  // Serial.print(Tmp / 340.00 + 36.53);
  // Serial.print(" | GyX = ");
  // Serial.print(GyX);
  // Serial.print(" | GyY = ");
  // Serial.print(GyY);
  // Serial.print(" | GyZ = ");
  // Serial.println(GyZ);

  led_state = !led_state;
  digitalWrite(LED_BUILTIN, led_state); // pisca LED do NodeMCU a cada leitura do sensor
  delay(50);
}

void getDirection()
{
  String url = "";

  if (AcX < -10000)
  {
    Serial.println("right");
    url = "/right";
  }
  if (AcX > 10000)
  {
    Serial.println("left");
    url = "/left";
  }
  if (AcY > 8000)
  {
    Serial.println("back");
    url = "/back";
  }
  if (AcY < -8000)
  {
    Serial.println("front");
    url = "/front";
  }

  if (url != "")
  {
    Serial.println("Requesting URL: ");
    Serial.print(url);

    // Use WiFiClient class to create TCP connections
    if (!client.connect(host, httpPort))
    {
      Serial.println("connection failed");
      return;
    }

    // This will send the request to the server
    client.print(String("GET ") + url + " HTTP/1.1\r\n" +
                 "Host: " + host + "\r\n" +
                 "Connection: close\r\n\r\n");
  }

  url = "";
}

/*
 * função de inicialização do sensor
 */
void initMPU()
{
  setSleepOff();
  setGyroScale();
  setAccelScale();
}

void setup()
{
  delay(1000);

  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(115200);

  // WIFI
  WiFi.mode(WIFI_STA);
  WiFiMulti.addAP("debarba-robot", "qwert12345");

  Serial.println();
  Serial.println();
  Serial.print("Wait for WiFi... ");

  while (WiFiMulti.run() != WL_CONNECTED)
  {
    Serial.print(".");
    delay(500);
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  delay(1000);

  // I2C
  Serial.println("Iniciando configuração do MPU6050");
  initI2C();
  initMPU();
  checkMPU(MPU_ADDR);

  Serial.println("Configuração finalizada, iniciando loop");
}

void loop()
{
  readRawMPU();
  getDirection();
  delay(50);
}