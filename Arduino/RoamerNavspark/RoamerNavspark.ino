//https://www.navsparkforum.com.tw/viewtopic.php?f=23&t=159&p=542&hilit=uart#p542
//U08	gnss_uart_rx_receive( U08 uart );

/* upload procedure if it's borked:
Plug in USB
Link BOOT_SEL to RF_GND
Press reset button
upload code
un link
reset
*/

uint8_t led = 13;


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
  gnss_uart_putline(0, (U08*) buf, len);

  if (Serial.available() > 0)
  {
    while (Serial.available() > 0) {
      int red = Serial.parseInt();
      int green = Serial.parseInt();
      int blue = Serial.parseInt();
      if (Serial.read() == '\n') {
        //act on it
        len = sprintf(buf, "R:%03d G:%03d B:%03d\r\n", red, green, blue);
        gnss_uart_putline(0, (U08*) buf, len);
      }
    }
    str = Serial.readStringUntil('\n');
          const char* string1 = str.c_str();
          len = sprintf(buf, string1);
          gnss_uart_putline(0, (U08*) buf, len);
  }

  delay(600);
}


void task_called_after_GNSS_update(void)
{
  static uint8_t val = HIGH;
  char buf[32];
  int len;

  if (val == HIGH) {
    digitalWrite(led, HIGH);
    len = sprintf(buf, "Hello world (GPIO%02d output 1)\r\n", led);
    gnss_uart_putline(0, (U08*) buf, len);
    val = LOW;
  }

  else if (val == LOW) {
    digitalWrite(led, LOW);
    len = sprintf(buf, "Hello world (GPIO%02d output 0)\r\n", led);
    gnss_uart_putline(0, (U08*) buf, len);
    val = HIGH;
  }

}
