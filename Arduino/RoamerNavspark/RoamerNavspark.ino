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

/*
Protocol: <LEDstartaddr,LEDlength,R,G,B>
e.g. <0,100,255,255,255>
*/

const byte numChars = 32;
char receivedChars[numChars];
char tempChars[numChars];        // temporary array for use when parsing
// variables to hold the parsed data
int LEDstartaddr = 0;
int LEDlength = 0;
int LEDred = 0;
int LEDgreen = 0;
int LEDblue = 0;
boolean newData = false;

//sonar shiz
byte ds[3];
unsigned long distance = 0;
int offst;
#define TCAADDR 0x70 //!< Default address of the multiplexer 0X70.
#define NSENSORS 2 //number of sonar modules


void setup() {
  GnssConf.init();
  // put your setup code here, to run once:
  Serial.config(STGNSS_UART_8BITS_WORD_LENGTH, STGNSS_UART_1STOP_BITS, STGNSS_UART_NOPARITY);
  Serial.begin(115200);

  nvsprkSerialPrintln("Running from FLASH");

  twMaster.config(100000); // 100KHz
  twMaster.begin();
  twMaster.setTransmitDeviceAddr(0x57); // 0x57 is the device address
  twMaster.setReceiveDeviceAddr(0x57); // 0x57 is the device address
}

void loop()
{
  // put your main code here, to run repeatedly:

  //handle comms
  recvWithStartEndMarkers();
  if (newData == true) {
    strcpy(tempChars, receivedChars);
    // this temporary copy is necessary to protect the original data
    //   because strtok() used in parseData() replaces the commas with \0
    parseLED();
    showLEDData();
    newData = false;
  }

  //handle sonar
  for (int addr = 0; addr <= NSENSORS - 1; addr++) {
    char buf[32];
    int len;
    int angle = 360/NSENSORS;
    sprintf(buf, "$SONAR,%d,%d", angle*addr, getSonar(addr));
    len = sprintf(buf, "%s*%02X\r\n", buf, nmea0183_checksum(buf));
    gnss_uart_putline(0, (U08*) buf, len);
  }

}

int getSonar(int addr) {
  hcselect(addr);
  delay(50); //wait for device to finish
  //ping device
  twMaster.beginTransmission(0x57);
  twMaster.write(1); //1 = cmd to start meansurement
  twMaster.endTransmission();
  delay(120); //wait for device to finish

  twMaster.requestFrom(0x57, 3);
  offst = 0;
  while (twMaster.available())
  {
    ds[offst++] = twMaster.read();
  }
  distance = (unsigned long)(ds[0]) * 65536;
  distance = distance + (unsigned long)(ds[1]) * 256;
  distance = (distance + (unsigned long)(ds[2])) / 10000;
  return distance;
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


void recvWithStartEndMarkers() {
  static boolean recvInProgress = false;
  static byte ndx = 0;
  char startMarker = '<';
  char endMarker = '>';
  char rc;

  while (Serial.available() > 0 && newData == false) {
    rc = Serial.read();

    if (recvInProgress == true) {
      if (rc != endMarker) {
        receivedChars[ndx] = rc;
        ndx++;
        if (ndx >= numChars) {
          ndx = numChars - 1;
        }
      }
      else {
        receivedChars[ndx] = '\0'; // terminate the string
        recvInProgress = false;
        ndx = 0;
        newData = true;
      }
    }

    else if (rc == startMarker) {
      recvInProgress = true;
    }
  }
}

//============

void parseLED() {      // split the data into its parts

  char * strtokIndx; // this is used by strtok() as an index

  strtokIndx = strtok(tempChars, ",");     // get the first part - the string
  LEDstartaddr = atoi(strtokIndx);  // copy it

  strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
  LEDlength = atoi(strtokIndx);     // convert this part to an integer

  strtokIndx = strtok(NULL, ",");
  LEDred = atoi(strtokIndx);     // convert this part to a float

  strtokIndx = strtok(NULL, ",");
  LEDgreen = atoi(strtokIndx);     // convert this part to a float

  strtokIndx = strtok(NULL, ",");
  LEDblue = atoi(strtokIndx);     // convert this part to a float

}

//============

void showLEDData() {

  char buf[32];

  nvsprkSerialPrint("LEDstartaddr  ");
  sprintf(buf, "%d", LEDstartaddr);
  nvsprkSerialPrintln(buf);

  nvsprkSerialPrint("LEDlength ");
  sprintf(buf, "%d", LEDlength);
  nvsprkSerialPrintln(buf);

  nvsprkSerialPrint("RGB Values ");
  sprintf(buf, "R%d, G%d, B%d", LEDred , LEDgreen , LEDblue);
  nvsprkSerialPrintln(buf);

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
