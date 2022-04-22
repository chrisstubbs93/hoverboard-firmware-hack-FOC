//https://www.navsparkforum.com.tw/viewtopic.php?f=23&t=159&p=542&hilit=uart#p542
//comms from https://forum.arduino.cc/t/serial-input-basics-updated/382007/3
//HC04 i2c from https://github.com/SGBotic/SGBotic_I2CPing

/* upload procedure if it's borked:
Plug in USB
Link BOOT_SEL to RF_GND
Press reset button
upload code
un link
reset
*/
#define TCAADDR 0x70 //!< Default address of the multiplexer 0X70.
#define NSENSORS 4 //number of sonar modules
#define NAVGS 3 //number of pings to average
#define FRONTSWPIN 12 //front bump sw
#define REARSWPIN 13 //rear bump sw

//sonar vars
byte ds[3];
unsigned long distanceraw[NSENSORS];
unsigned long distance[NSENSORS];
int nreadings[NSENSORS];

//for sending serial
char buf[32];
int len;

void setup() {
  GnssConf.init();
  // put your setup code here, to run once:
  Serial.config(STGNSS_UART_8BITS_WORD_LENGTH, STGNSS_UART_1STOP_BITS, STGNSS_UART_NOPARITY);
  Serial.begin(115200);
  nvsprkSerialPrintln("$STATUS,Running_from_FLASH*00");

  pinMode(FRONTSWPIN, INPUT);
  pinMode(REARSWPIN, INPUT);

  twMaster.config(50000); // 50KHz
  twMaster.begin();
  twMaster.setTransmitDeviceAddr(0x57); // 0x57 is the device address
  twMaster.setReceiveDeviceAddr(0x57); // 0x57 is the device address
}

void loop()
{
  //handle sonar
  getSonar();
  sprintf(buf, "$SONAR");
  for (int addr = 0; addr <= NSENSORS - 1; addr++) {
    int angle = 360 / NSENSORS;
    sprintf(buf, "%s,%d:%d", buf, angle * addr, distance[addr]);
  }
  len = sprintf(buf, "%s*%02X\r\n", buf, nmea0183_checksum(buf));
  gnss_uart_putline(0, (U08*) buf, len);

  //handle switches
  sprintf(buf, "$BUMP,%d,%d", 0, digitalRead(FRONTSWPIN));
  len = sprintf(buf, "%s*%02X\r\n", buf, nmea0183_checksum(buf));
  gnss_uart_putline(0, (U08*) buf, len);

  sprintf(buf, "$BUMP,%d,%d", 180, digitalRead(REARSWPIN));
  len = sprintf(buf, "%s*%02X\r\n", buf, nmea0183_checksum(buf));
  gnss_uart_putline(0, (U08*) buf, len);
}

int getSonar() {
  memset(nreadings, 0, sizeof(nreadings)); //clear the registers
  memset(distanceraw, 0, sizeof(distanceraw)); //clear the registers

  for (int r = 0; r <= NAVGS-1; r++) { //repeats for avg
    hcsetall(); //send to each sub-busses
    sonarPing(); //PING
    for (int addr = 0; addr <= NSENSORS - 1; addr++) {
      unsigned long pingResult = sonarGetResult(addr);
      if ((1 <= pingResult) && (900 >= pingResult)) //measured value between 1cm to 9meters
      {
        distanceraw[addr] = distanceraw[addr] + pingResult;
        nreadings[addr]++;
      }
    }
  }
  for (int addr = 0; addr <= NSENSORS - 1; addr++) {
    if (nreadings[addr] > 0) {
      distance[addr] = distanceraw[addr] / nreadings[addr];
    }
    else {
      distance[addr] = 999;
    }
  }
}

void sonarPing() {
  twMaster.beginTransmission(0x57);
  twMaster.write(1); //1 = cmd to start meansurement
  twMaster.endTransmission();
  delay(110); //wait for device to finish
}
unsigned long sonarGetResult(uint8_t addr) {
  hcselect(addr);
  twMaster.requestFrom(0x57, 3);
  memset(ds, 0, sizeof(ds)); //clear the registers
  for (int o = 0; o <= 2; o++)
  {
    ds[o] = twMaster.read();
  }
  unsigned long temp;
  temp = (unsigned long)(ds[0]) * 65536;
  temp = temp + (unsigned long)(ds[1]) * 256;
  temp = (temp + (unsigned long)(ds[2])) / 10000;
  return temp;
}

void nvsprkSerialPrint(char msgToSend[]) {
  char buf[32];
  int len;
  len = sprintf(buf, "%s", msgToSend);
  gnss_uart_putline(0, (U08*) buf, len);
}
void nvsprkSerialPrintln(char msgToSend[]) {
  char buf[32];
  int len;
  len = sprintf(buf, "%s\r\n", msgToSend);
  gnss_uart_putline(0, (U08*) buf, len);
}



void task_called_after_GNSS_update(void)
{
  //nothing
}

/*! Selects which i2c sub-bus on the mux for a specific sensor to connect to and checks it, retrying if it's not correct. (input 0...7) */
void hcselect(uint8_t i) {
  if (i > 7) return;
  do {
    twMaster.beginTransmission(TCAADDR);
    twMaster.write(1 << i);
    twMaster.endTransmission();
  } while (!(hcget() == i));
  delay(20); //wait for device to switch
}

/*! Check the current i2c sub-bus for a specific sensor. Output 0...7 */
int hcget() {
  twMaster.requestFrom(TCAADDR, 1);
  byte c = twMaster.read();
  for (int i = 0; i <= 7; i++) {
    if (c == 0x01) return i;
    c = c >> 1;
  }
}

void hcsetall() {
  do {
    twMaster.beginTransmission(TCAADDR);
    twMaster.write(0xFF);
    twMaster.endTransmission();
  } while (!(hcgetall() == 0xFF));
  delay(20); //wait for device to switch
}

int hcgetall() {
  twMaster.requestFrom(TCAADDR, 1);
  byte c = twMaster.read();
  return c;
}


int nmea0183_checksum(char *nmea_data)
{
  int crc = 0;
  int i;
  // ignore the first $ sign,  no checksum in sentence
  for (i = 1; i < strlen(nmea_data); i ++) { // removed the - 3 because no cksum is present
    crc ^= nmea_data[i];
  }
  return crc;
}
