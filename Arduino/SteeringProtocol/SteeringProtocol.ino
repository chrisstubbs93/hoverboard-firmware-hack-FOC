#define START_FRAME         0xABCD       // [-] Start frme definition for reliable serial communication

// Global variables
uint8_t idx = 0;                        // Index for new data pointer
uint16_t bufStartFrame;                 // Buffer Start Frame
byte *p;                                // Pointer declaration for the new received data
byte incomingByte;
byte incomingBytePrev;

typedef struct{
   uint16_t start;
   int16_t  steer;
   uint16_t checksum;
} SteerCmd;
SteerCmd SCmd;
SteerCmd SnewCmd;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly:
Receive();
}

void Receive()
{
    // Check for new data availability in the Serial buffer
    if (Serial.available()) {
        incomingByte     = Serial.read();// Read the incoming byte
        bufStartFrame = ((uint16_t)(incomingByte) << 8) | incomingBytePrev;// Construct the start frame
    }
    else {
        return;
    }
    // Copy received data
    if (bufStartFrame == START_FRAME) {// Initialize if new data is detected
        p       = (byte *)&SnewCmd;
        *p++    = incomingBytePrev;
        *p++    = incomingByte;
        idx     = 2;  
    } else if (idx >= 2 && idx < sizeof(SteerCmd)) {  // Save the new received data
        *p++    = incomingByte; 
        idx++;
    } 
    // Check if we reached the end of the package
    if (idx == sizeof(SteerCmd)) {
        uint16_t checksum;
        checksum = (uint16_t)(SnewCmd.start ^ SnewCmd.steer);
        // Check validity of the new data
        if (SnewCmd.start == START_FRAME && checksum == SnewCmd.checksum) {
            // Copy the new data
            memcpy(&Cmd, &SnewCmd, sizeof(SteerCmd));
            // Print data to built-in Serial
            //Serial.print("1: ");   Serial.print(Cmd.steer);
        } else {
          //Serial.println("Non-valid data skipped");
        }
        idx = 0;    // Reset the index (it prevents to enter in this if condition in the next cycle)
    }
    // Update previous states
    incomingBytePrev = incomingByte;
}
