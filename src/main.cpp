
/*
 * Teensy 4.1 crude logger
 yoinked from hytech racing "telemetry control unit" repo
 removed all features except can logging to SD
 */
#include <Arduino.h>
#include <SD.h>
#include <Wire.h>
#include <TimeLib.h>
#include <Metro.h>
#include <FlexCAN_T4.h>
#include <RadioLib.h>
int pin_cs = 10;
int pin_dio0 = 6;
int pin_nrst = 7;
int pin_dio1 = 5;
SX1276 radio = new Module(pin_cs, pin_dio0, pin_nrst, pin_dio1);
float packvoltage=0;
float motortemp=0;
float invertertemp=0;
String invfaults="";
/*
 * CAN Variables
 */
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> CAN;
static CAN_message_t msg_rx;
static CAN_message_t msg_tx;
// static CAN_message_t xb_msg;
File logger;
String output;
/*
 * Variables to help with time calculation
 */
uint64_t global_ms_offset = 0;
uint64_t last_sec_epoch;
Metro timer_debug_RTC = Metro(1000);
Metro timer_flush = Metro(50);
Metro LoraTimer = Metro(100);
Metro getLoraData = Metro(200);
void parse_can_message();
void write_to_SD(CAN_message_t *msg);
time_t getTeensy3Time();
void sd_date_time(uint16_t* date, uint16_t* time);
void setup() {
  delay(500); //Wait for ESP32 to be able to print

  Serial.print(F("[SX1276] Initializing ... "));
  //int state = radio.begin(); //-121dBm
  //int state = radio.begin(868.0); //-20dBm
  int state = radio.begin(915.0); //-23dBm
  if (state == RADIOLIB_ERR_NONE) {
    Serial.println(F("init success!"));
  } else {
    Serial.print(F("failed, code "));
    Serial.println(state);
    while (true);
  }

  // set output power to 10 dBm (accepted range is -3 - 17 dBm)
  // NOTE: 20 dBm value allows high power operation, but transmission
  //       duty cycle MUST NOT exceed 1%
//  if (radio.setOutputPower(20) == ERR_INVALID_OUTPUT_POWER) {
//    Serial.println(F("Selected output power is invalid for this module!"));
//    while (true);
//  }

  // some modules have an external RF switch
  // controlled via two pins (RX enable, TX enable)
  // to enable automatic control of the switch,
  // call the following method
  int pin_rx_enable = 8;
  int pin_tx_enable = 9;
  radio.setRfSwitchPins(pin_rx_enable, pin_tx_enable);
    //SD logging init stuff
    delay(500); // Prevents suprious text files when turning the car on and off rapidly
    /* Set up Serial, CAN */
    //Serial.begin(115200);

    /* Set up real-time clock */
    //Teensy3Clock.set(1660351622); // set time (epoch) at powerup  (COMMENT OUT THIS LINE AND PUSH ONCE RTC HAS BEEN SET!!!!)
    setSyncProvider(getTeensy3Time); // registers Teensy RTC as system time
    if (timeStatus() != timeSet) {
        Serial.println("RTC not set up - uncomment the Teensy3Clock.set() function call to set the time");
    } else {
        Serial.println("System time set to RTC");
    }
    last_sec_epoch = Teensy3Clock.get();
    
    //FLEXCAN0_MCR &= 0xFFFDFFFF; // Enables CAN message self-reception
    CAN.begin();
    CAN.setBaudRate(1000000);
    /* Set up SD card */
    Serial.println("Initializing SD card...");
    SdFile::dateTimeCallback(sd_date_time); // Set date/time callback function
    if (!SD.begin(BUILTIN_SDCARD)) { // Begin Arduino SD API (Teensy 3.5)
        Serial.println("SD card failed or not present");
    }
    char filename[] = "data0000.CSV";
    for (uint8_t i = 0; i < 10000; i++) {
        filename[4] = i / 1000     + '0';
        filename[5] = i / 100 % 10 + '0';
        filename[6] = i / 10  % 10 + '0';
        filename[7] = i       % 10 + '0';
        if (!SD.exists(filename)) {
            logger = SD.open(filename, (uint8_t) O_WRITE | (uint8_t) O_CREAT); // Open file for writing
            break;
        }
        if (i == 9999) { // If all possible filenames are in use, print error
            Serial.println("All possible SD card log filenames are in use - please clean up the SD card");
        }
    }
    
    if (logger) {
        Serial.print("Successfully opened SD file: ");
        Serial.println(filename);
    } else {
        Serial.println("Failed to open SD file");
    }
    
    logger.println("time,msg.id,msg.len,data"); // Print CSV heading to the logfile
    logger.flush();
}
void loop() {
    if(CAN.read(msg_rx)){
        // if(msg_rx.id==0xA7){
        //     packvoltage=(msg_rx.buf[0]+msg_rx.buf[1]*256)/10;
        //     Serial.printf("PackVolts: %f\n",packvoltage);
        // }
        // if(msg_rx.id==0xA2){
        //     motortemp=(msg_rx.buf[4]+(msg_rx.buf[5]*256))/10;
        //     Serial.printf("motortemp: %f\n",motortemp);
        // }
        // if(msg_rx.id==0xA0){
        //     invertertemp=(msg_rx.buf[0]+(msg_rx.buf[1]*256))/10;
        //     Serial.printf("invtemp: %f\n",invertertemp);
        // }
        // if(msg_rx.id==0xAB){
        //     invfaults="";
        //     for(int i = 0; i<msg_rx.len;i++){
        //         invfaults+=String(msg_rx.buf[i],HEX);
        //     }
        // }
    }
    /* Process and log incoming CAN messages */
    parse_can_message();
    /* Flush data to SD card occasionally */
    if (timer_flush.check()) {
        logger.flush(); // Flush data to disk (data is also flushed whenever the 512 Byte buffer fills up, but this call ensures we don't lose more than a second of data when the car turns off)
    }
    /* Print timestamp to serial occasionally */
    if (timer_debug_RTC.check()) {
        Serial.println(Teensy3Clock.get());
    }


    if(LoraTimer.check()){
        float starttime = millis();
        output="";
        output = String(packvoltage);
        output+=",";
        output+=String(motortemp);
        output+=",";
        output+=String(invertertemp);
        output+=",";
        output+=invfaults;        
        int state = radio.transmit(output);
        if (state == RADIOLIB_ERR_NONE) {
            // the packet was successfully transmitted
            Serial.println(F(" success!"));

            // print measured data rate
            Serial.print(F("[SX1276] Datarate:\t"));
            Serial.print(radio.getDataRate());
            Serial.println(F(" bps"));

        } else if (state == RADIOLIB_ERR_PACKET_TOO_LONG) {
            // the supplied packet was longer than 256 bytes
            Serial.println(F("too long!"));

        } else if (state == RADIOLIB_ERR_TX_TIMEOUT) {
            // timeout occurred while transmitting packet
            Serial.println(F("timeout!"));

        } else {
            // some other error occurred
            Serial.print(F("failed, code "));
            Serial.println(state);
        }
        float endtime = millis();
    Serial.printf("took this long %f\n",endtime-starttime);

}
}
void parse_can_message() {
    while (CAN.read(msg_rx)) {

        write_to_SD(&msg_rx); // Write to SD card buffer (if the buffer fills up, triggering a flush to disk, this will take 8ms)
        if(msg_rx.id==0xA7){
            packvoltage=(msg_rx.buf[0]+msg_rx.buf[1]*256)/10;
            Serial.printf("PackVolts: %f\n",packvoltage);
        }
        if(msg_rx.id==0xA2){
            motortemp=(msg_rx.buf[4]+(msg_rx.buf[5]*256))/10;
            Serial.printf("motortemp: %f\n",motortemp);
        }
        if(msg_rx.id==0xA0){
            invertertemp=(msg_rx.buf[0]+(msg_rx.buf[1]*256))/10;
            Serial.printf("invtemp: %f\n",invertertemp);
        }
        if(msg_rx.id==0xAB){
            invfaults="";
            for(int i = 0; i<msg_rx.len;i++){
                invfaults+=String(msg_rx.buf[i],HEX);
            }
        }
        
    }
}
void write_to_SD(CAN_message_t *msg) { // Note: This function does not flush data to disk! It will happen when the buffer fills or when the above flush timer fires
    // Calculate Time
    //This block is verified to loop through

    uint64_t sec_epoch = Teensy3Clock.get();
    if (sec_epoch != last_sec_epoch) {
        global_ms_offset = millis() % 1000;
        last_sec_epoch = sec_epoch;
    }
    uint64_t current_time = sec_epoch * 1000 + (millis() - global_ms_offset) % 1000;

    // Log to SD
    logger.print(current_time);
    logger.print(",");
    logger.print(msg->id, HEX);
    logger.print(",");
    logger.print(msg->len);
    logger.print(",");
    for (int i = 0; i < msg->len; i++) {
        if (msg->buf[i] < 16) {
            logger.print("0");
        }
        logger.print(msg->buf[i], HEX);
    }
    logger.println();
}
time_t getTeensy3Time() {
    return Teensy3Clock.get();
}
void sd_date_time(uint16_t* date, uint16_t* time) {
    // return date using FAT_DATE macro to format fields
    *date = FAT_DATE(year(), month(), day());
    // return time using FAT_TIME macro to format fields
    *time = FAT_TIME(hour(), minute(), second());
}