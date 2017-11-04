#include <ModbusRtu.h>
#include <SoftwareSerial.h>

//Arrays de dados a receber
uint16_t au16dataRX[16];
//Arrays de dados a transmitir
uint16_t au16dataTX[16];
uint8_t byteTX[32];
uint8_t u8state;

/**
    Modbus object declaration
    u8id : node id = 0 for master, = 1..247 for slave
    u8serno : serial port (use 0 for Serial)
    u8txenpin : 0 for RS-232 and USB-FTDI
                 or any pin number > 1 for RS-485
*/
Modbus master(0, 1, 2);

//Funções Modbus
modbus_t readRegisters;
modbus_t writeRegisters;

//Porta Bluetooth
SoftwareSerial mySerial(10, 11); // RX, TX

unsigned long u32wait;

void setup() {

  //read registers
  readRegisters.u8id = 1;                 // slave address
  readRegisters.u8fct = 3;                // function code (registers read multiple  3)
  readRegisters.u16RegAdd = 0;            // start address in slave -  direccion de Inicio 0
  readRegisters.u16CoilsNo = 2;          // number of elements (coils or registers) to read  0 - 16
  readRegisters.au16reg = au16dataRX;       // pointer to a memory array in the Arduino - Almacenamiento en Array de memoria de arduino

  //write a multiple  register = function 16
  writeRegisters.u8id = 1;                 // slave address
  writeRegisters.u8fct = 16;               // function code (write multiple registers 16 )
  writeRegisters.u16RegAdd = 1;           // start address in slave  -  direccion de Inicio 10
  writeRegisters.u16CoilsNo = 1;          // number of elements (coils or registers) to read
  writeRegisters.au16reg = au16dataTX;

  Serial.begin(9600);
  master.begin( 9600 ); // baud-rate at 9600
  mySerial.begin(9600);
  master.setTimeOut( 2000 ); // if there is no answer in 2000 ms, roll over
  u32wait = millis() + 1000;
}

void loop() {
  switch ( u8state ) {
    case 0:
      if (millis() > u32wait)
        u8state++; // wait state
      break;
    case 1:
      master.query( readRegisters );
      u8state++;
      break;
    case 2:
      master.poll(); // check incoming messages
      if (master.getState() == COM_IDLE) {
        byteTX[0] = highByte(au16dataRX[0]);
        byteTX[1] = lowByte(au16dataRX[0]);
        byteTX[2] = highByte(au16dataRX[1]);
        byteTX[3] = lowByte(au16dataRX[1]);

        if (!master.getTimeOutState()) {
          mySerial.write(byteTX[0]);
          mySerial.write(byteTX[1]);
          mySerial.write(byteTX[2]);
          mySerial.write(byteTX[3]);
        }
        else {
          mySerial.write("!");
          mySerial.write("!");
          mySerial.write("!");
          mySerial.write("!");
          mySerial.write("!");
        }
        u8state = 0;
        u32wait = millis() + 1000;
      }
      break;
  }

  if (mySerial.available() > 0) {
    String str;
    while (mySerial.available() > 0) {
      int inChar = mySerial.read();
      delayMicroseconds(150);
      str += String(inChar);
    }
    au16dataTX[0] = str.toInt();
    master.query( writeRegisters );
  }
  master.poll();
}
