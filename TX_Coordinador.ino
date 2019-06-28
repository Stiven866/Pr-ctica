#include <LiquidCrystal_I2C.h>
#include <Keypad.h>

#define selector 2

const byte Filas = 4;          //KeyPad de 4 filas
const byte Cols = 4;           //y 4 columnas

byte Pins_Filas[] = {11, 10, 9, 8};     //Pines Arduino para las filas.
byte Pins_Cols[] = { 7, 6, 5, 4};     // Pines Arduino para las columnas.

char Teclas [ Filas ][ Cols ] =
{
  {'1', '2', '3', 'A'},
  {'4', '5', '6', 'B'},
  {'7', '8', '9', 'C'},
  {'*', '0', '#', 'D'}
};

Keypad Teclado1 = Keypad(makeKeymap(Teclas), Pins_Filas, Pins_Cols, Filas, Cols);
LiquidCrystal_I2C lcd(0x27, 16, 2);



int lengthFrameData = 0;
byte sourceAddress[8];
byte dataBytes[255];
byte XBeeNivel_1[8] = {0x00, 0x13, 0xA2, 0x00, 0x40, 0xE6, 0x73, 0x93};
byte XbeeHuerta_2[8] = {0x00, 0x13, 0xA2, 0x00, 0x40, 0x6E, 0x84, 0x0B};
char val = '0';
byte temperatura[2], humedadSuelo[2], humedadRelativa[2];
bool nivel_Bajo = false, nivel_Alto = false, estadoRiego = false;
int temp = 0;
int humRel = 0;
int humSuelo, hs = 100;
int var = 0;
int type_frame = 0;


void setup() {

  pinMode(selector, INPUT);
  Serial1.begin(9600);
  Serial.begin(19200);
  lcd.init();
  lcd.backlight();
  lcd.setCursor(5, 0);
  lcd.print("Huerta");
  lcd.setCursor(1, 1);
  lcd.print("Antonio Derka");
  setRemoteState(0x04);
  delay(2000);
  setRemoteState_v(0x04);
  delay(2000);
  lcd.clear();
}

void loop () {
  if (Serial1.available() >= 21) {
    leerSerial();
  }

  if (type_frame == 0x91) {
    for (int i = 0; i < 8; i++) {
      if (XBeeNivel_1[i] == sourceAddress[i]) {
        val = 'a';
        continue;
      }
      if (XbeeHuerta_2[i] == sourceAddress[i]) {
        val = 'b';
        continue;
      }
    }
    valor_Nivel(dataBytes);
    valores_Sensores(dataBytes);
  }

  //Manual
  if (digitalRead(selector) == LOW) {
    char pulsacion = Teclado1.getKey() ;
    if (pulsacion != 0) {             // Si el valor es 0 es que no se
      if (pulsacion == '*') {
        setRemoteState_v(0x05);
      } else if (pulsacion == '#') {
        setRemoteState_v(0x04);
      }
    }
    if (nivel_Alto == true && nivel_Bajo == true) {
      setRemoteState_v(0x04);
    }
    lcd.clear();
    lcd.setCursor(2, 0);
    lcd.print("Modo Manual");
  } else {
    switch (val) {
      case 'a':
        //11 -> vacio      10 -> medio del tanque
        if ((nivel_Alto == true && nivel_Bajo == true) || (nivel_Alto == true && nivel_Bajo == false) ) {
          //Serial.println("Nivel Bajo");
          lcd.setCursor(9, 1);
          lcd.print("N Bajo");
          if (estadoRiego == true) {
            setRemoteState(0x04);
            if ((nivel_Alto == true && nivel_Bajo == true)) {
              setRemoteState_v(0x04);
              estadoRiego = false;
            }
          } else {
            setRemoteState(0x05);
          }

        }
        //00 -> lleno
        else if ((nivel_Alto == false && nivel_Bajo == false)) {
          //Serial.println("Nivel Alto");
          lcd.setCursor(9, 1);
          lcd.print("N Alto");
          setRemoteState(0x04);
          if (hs < 40) {
            setRemoteState_v(0x05);
            estadoRiego = true;//Está regando
          }
        }


        val = '0';
        break;
      case 'b':


        //Humedad Relativa
        lcd.setCursor(7, 0);
        lcd.print("  HS:");
        lcd.print(humRel, HEX);
        lcd.print("%");

        //Temperatura
        lcd.setCursor(0, 1);
        lcd.print("T:");
        var = (temp / 0.364) * (1.2 * 100) / 1024.0;
        lcd.print(var);
        lcd.print("C");

        //Humedad Suelo
        hs = map((humSuelo), 0, 1023, 100, 0);
        lcd.setCursor(0, 0);
        lcd.print("HS:");
        lcd.print(hs);
        lcd.print("%");
        val = '0';
        break;
    }


  }




}


void leerSerial() {
  if (startDelimiter()) {
    Serial.print("length frame: ");
    lengthFrameData = lengthFrame(true) - 18;
    Serial.println(lengthFrameData, HEX);
    Serial.print("type frame: ");
    type_frame = typeFrame();
    Serial.println(type_frame, HEX);

    addressOfSender(sourceAddress);
    printAddress(sourceAddress);
    sourceNetworkAddress();
    sourceEndPoint();
    destinationEndPoint();
    clusterID();
    profileID();
    receiveOptions();
    data(lengthFrameData, dataBytes);
    checkSum();
  }
}
void valores_Sensores(byte data[]) {
  temp = data[6];
  temp = (temp << 8) + data[7];
  humRel = data[4];
  humRel = (humRel << 8) + data[5];
  humSuelo = data[8];
  humSuelo = (humSuelo << 8) + data[9];
}

void valor_Nivel(byte array1[]) {

  if ((0x01) == (array1[5] & 0x01)) {
    nivel_Bajo = true;
  } else {
    nivel_Bajo = false;
  }

  if ((0x02) == (array1[5] & 0x02)) {
    nivel_Alto = true;
  } else {
    nivel_Alto = false;
  }

}


bool startDelimiter() {
  if (Serial1.read() == 0x7E) {
    return true;
  }
  return false;
}

/*Number of vytes between the length and the checksum*/
int lengthFrame(bool delimiter) {
  if (delimiter) {
    int MSB1 = Serial1.read();
    int LSB2 = Serial1.read();
    MSB1 = MSB1 << 8;
    int lengthBytes = MSB1 + LSB2;
    //Serial.println(lengthBytes);
    return lengthBytes;
  }
  return 0;
}

int typeFrame() {
  return Serial1.read();
}

void addressOfSender(byte address[]) {
  for (int i = 0; i < 8; i++) {
    address[i] = Serial1.read();
  }
}

void printAddress(byte address[]) {
  Serial.print("Address: ");
  for (int i = 0; i < 8; i++) {
    Serial.print(address[i], HEX);
  }
  Serial.println();
}

void sourceNetworkAddress() {
  int sourceNetworkAddressMSB = Serial1.read();
  int sourceNetworkAddressLSB = Serial1.read();
  Serial.print("sourceNetworkAddress: ");
  Serial.print(sourceNetworkAddressMSB, HEX);
  Serial.println(sourceNetworkAddressLSB, HEX);
}

void sourceEndPoint() {
  int sourceEP = Serial1.read();
  Serial.print("sourceEndPoint: ");
  Serial.println(sourceEP, HEX);
}

void destinationEndPoint() {
  int destinationEP = Serial1.read();
  Serial.print("destinationEndPoint: ");
  Serial.println(destinationEP, HEX);
}


void clusterID() {
  int clusterIDMSB = Serial1.read();
  int clusterIDLSB = Serial1.read();
  Serial.print("clusterID: ");
  Serial.print(clusterIDMSB, HEX);
  Serial.println(clusterIDLSB, HEX);
}

void profileID() {
  int profileIDMSB = Serial1.read();
  int profileIDLSB = Serial1.read();
  Serial.print("profileID: ");
  Serial.print(profileIDMSB, HEX);
  Serial.println(profileIDLSB, HEX);
}

void receiveOptions() {
  int receiveOpt = Serial1.read();
  Serial.print("receiveOptions: ");
  Serial.println(receiveOpt, HEX);
}


void data(int lengthData, byte dataByte[] ) {
  Serial.print("data: ");
  for (int i = 0; i < lengthData; i++) {
    dataByte[i] = Serial1.read();
    Serial.print(dataByte[i], HEX);
  }
  Serial.println();
}

void checkSum() {
  int checkAdd = Serial1.read();
  Serial.print("checkSum: ");
  Serial.println(checkAdd, HEX);
}


void setRemoteState(char value) {
  Serial1.write(0x7E); /*Delimitador de inicio*/
  Serial1.write((byte)0x0);/*Parte alta de la longitud (siempre cero)*/
  Serial1.write(0x10);/*Parte baja de la longitud desde acá hasta antes del checksum*/
  Serial1.write(0x17);/*Comando AT remoto*/
  Serial1.write((byte)0x01);/*Frame ID*/
  Serial1.write((byte)0x0);/*Dirección 64 bits*/
  Serial1.write(0x13);/**/
  Serial1.write(0xA2);/**/
  Serial1.write(0x0);/**/
  Serial1.write(0x40);/**/
  Serial1.write(0xE6);/**/
  Serial1.write(0x73);/**/
  Serial1.write(0x93);/**/
  Serial1.write(0xFF);/**/
  Serial1.write(0xFE);/**/
  Serial1.write(0x02);/*Aplicar cambios*/
  Serial1.write('D');/*Puerto*/
  Serial1.write('2');
  Serial1.write(value);
  long sum = 0x17 + 0x01 + 0x13 + 0xA2 + 0x40 + 0xE6 + 0x73 + 0x93 + 0xFF + 0xFE + 0x02 + 'D' + '2' + value;
  long checksum = 0xFF - (sum & 0xFF);
  //Serial.println(checksum, HEX);
  Serial1.write(checksum);
}

void setRemoteState_v(char value) {
  Serial1.write(0x7E); /*Delimitador de inicio*/
  Serial1.write((byte)0x0);/*Parte alta de la longitud (siempre cero)*/
  Serial1.write(0x10);/*Parte baja de la longitud desde acá hasta antes del checksum*/
  Serial1.write(0x17);/*Comando AT remoto*/
  Serial1.write((byte)0x01);/*Frame ID*/
  Serial1.write((byte)0x0);/*Dirección 64 bits*/
  Serial1.write(0x13);/**/
  Serial1.write(0xA2);/**/
  Serial1.write(0x0);/**/
  Serial1.write(0x40);/**/
  Serial1.write(0xE6);/**/
  Serial1.write(0x73);/**/
  Serial1.write(0x93);/**/
  Serial1.write(0xFF);/**/
  Serial1.write(0xFE);/**/
  Serial1.write(0x02);/*Aplicar cambios*/
  Serial1.write('D');/*Puerto*/
  Serial1.write('3');
  Serial1.write(value);
  long sum = 0x17 + 0x01 + 0x13 + 0xA2 + 0x40 + 0xE6 + 0x73 + 0x93 + 0xFF + 0xFE + 0x02 + 'D' + '3' + value;
  long checksum = 0xFF - (sum & 0xFF);
  //Serial.println(checksum, HEX);
  Serial1.write(checksum);
}
