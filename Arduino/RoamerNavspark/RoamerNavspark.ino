//https://www.navsparkforum.com.tw/viewtopic.php?f=23&t=159&p=542&hilit=uart#p542
//U08	gnss_uart_rx_receive( U08 uart );

//comms from https://forum.arduino.cc/t/serial-input-basics-updated/382007/3

/* upload procedure if it's borked:
Plug in USB
Link BOOT_SEL to RF_GND
Press reset button
upload code
un link
reset
*/

uint8_t led = 13;

const byte numChars = 32;
char receivedChars[numChars];
char tempChars[numChars];        // temporary array for use when parsing

// variables to hold the parsed data
char messageFromPC[numChars] = {0};
int integerFromPC = 0;
float floatFromPC = 0.0;

int LEDstartaddr = 0;
int LEDlength = 0;
int LEDred = 0;
int LEDgreen = 0;
int LEDblue = 0;

boolean newData = false;


void setup() {
  GnssConf.init();
  // put your setup code here, to run once:
  pinMode(led, OUTPUT);
  //Serial1.config(STGNSS_UART_8BITS_WORD_LENGTH, STGNSS_UART_1STOP_BITS, STGNSS_UART_NOPARITY);
  //Serial1.begin();

  Serial.config(STGNSS_UART_8BITS_WORD_LENGTH, STGNSS_UART_1STOP_BITS, STGNSS_UART_NOPARITY);
  Serial.begin(115200);
}

void loop()
{
  // put your main code here, to run repeatedly:
  char buf[32];
  int len;
  String str;
  len = sprintf(buf, "$SONAR,360,100*FF\r\n");
  //gnss_uart_putline(0, (U08*) buf, len);


  recvWithStartEndMarkers();
  if (newData == true) {
    strcpy(tempChars, receivedChars);
    // this temporary copy is necessary to protect the original data
    //   because strtok() used in parseData() replaces the commas with \0
    parseLED();
    showLEDData();
    newData = false;
  }


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

void parseData() {      // split the data into its parts

  char * strtokIndx; // this is used by strtok() as an index

  strtokIndx = strtok(tempChars, ",");     // get the first part - the string
  strcpy(messageFromPC, strtokIndx); // copy it to messageFromPC

  strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
  integerFromPC = atoi(strtokIndx);     // convert this part to an integer

  strtokIndx = strtok(NULL, ",");
  floatFromPC = atof(strtokIndx);     // convert this part to a float

}

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

void showParsedData() {
  
  char buf[32];
  
  nvsprkSerialPrint("Message ");
  nvsprkSerialPrintln(messageFromPC);
  nvsprkSerialPrint("Integer ");
  sprintf(buf, "%d", integerFromPC);
  nvsprkSerialPrintln(buf);
  nvsprkSerialPrint("Float ");
  sprintf(buf, "%f", floatFromPC);
  nvsprkSerialPrintln(buf);
}

void showLEDData() {
  
  char buf[32];
  
  nvsprkSerialPrint("LEDstartaddr  ");
  sprintf(buf, "%d", LEDstartaddr);
  nvsprkSerialPrintln(buf);
  
  nvsprkSerialPrint("LEDlength ");
  sprintf(buf, "%d", LEDlength);
  nvsprkSerialPrintln(buf);
  
  nvsprkSerialPrint("RGB Values ");
  sprintf(buf, "R%d, G%d, B%d", LEDred ,LEDgreen , LEDblue);
  nvsprkSerialPrintln(buf);

}


void task_called_after_GNSS_update(void)
{
  static uint8_t val = HIGH;
  char buf[32];
  int len;

  if (val == HIGH) {
    digitalWrite(led, HIGH);
    //len = sprintf(buf, "Hello world (GPIO%02d output 1)\r\n", led);
    //gnss_uart_putline(0, (U08*) buf, len);
    val = LOW;
  }

  else if (val == LOW) {
    digitalWrite(led, LOW);
    //len = sprintf(buf, "Hello world (GPIO%02d output 0)\r\n", led);
    //gnss_uart_putline(0, (U08*) buf, len);
    val = HIGH;
  }

}
