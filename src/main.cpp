
/*
 * Teensy 4.1 crude logger
 yoinked from hytech racing "telemetry control unit" repo
 removed all features except can logging to SD
 */
#include <KS2eCAN.hpp>
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
int transmissionState = RADIOLIB_ERR_NONE;

SX1276 radio = new Module(pin_cs, pin_dio0, pin_nrst, pin_dio1);
uint16_t packvoltage=0,invertercurrent=0,torquereq=0,motorrpm=0;
uint16_t motortemp=0;
uint16_t invertertemp=0;
uint16_t accel1_=0,accel2_=0,brake1_=0;
String invfaults="";
String pedalreadings="";
// flag to indicate that a packet was sent
volatile bool transmittedFlag = true;

// disable interrupt when it's not needed
volatile bool enableInterrupt = true;

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
Metro timer_flush = Metro(5);
Metro LoraTimer = Metro(100);
Metro getLoraData = Metro(200);
void parse_can_message();
void write_to_SD(CAN_message_t *msg);
time_t getTeensy3Time();
void sd_date_time(uint16_t* date, uint16_t* time);
void setFlag(void) {
  // check if the interrupt is enabled
  if(!enableInterrupt) {
    return;
  }

  // we sent a packet, set the flag
  transmittedFlag = true;
  
}
void setup() {
    pinMode(LED_BUILTIN,OUTPUT);
    digitalWrite(LED_BUILTIN,HIGH);

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
  radio.setDio0Action(setFlag);
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
    transmissionState = radio.startTransmit("yeet");
}

// this function is called when a complete packet
// is transmitted by the module
// IMPORTANT: this function MUST be 'void' type
//            and MUST NOT have any arguments!
void loop() {
    digitalWrite(LED_BUILTIN,LOW);
    if(CAN.read(msg_rx)){
        if(msg_rx.id==ID_MC_TORQUE_TIMER_INFORMATION){
            torquereq=(msg_rx.buf[0]+msg_rx.buf[1]*256)/10;
            // Serial.printf("torquereq: %d\n",torquereq);
        }        
        if(msg_rx.id==ID_MC_MOTOR_POSITION_INFORMATION){
            motorrpm=(msg_rx.buf[2]+msg_rx.buf[3]*256)/10;
            // Serial.printf("motorrpm: %d\n",motorrpm);
        }
        if(msg_rx.id==ID_MC_CURRENT_INFORMATION){
            invertercurrent=(msg_rx.buf[6]+msg_rx.buf[7]*256)/10;
            // Serial.printf("PackCurrent: %d\n",invertercurrent);
        }
        if(msg_rx.id==ID_MC_VOLTAGE_INFORMATION){
            packvoltage=(msg_rx.buf[0]+msg_rx.buf[1]*256)/10;
            // Serial.printf("PackVolts: %d\n",packvoltage);
        }
        if(msg_rx.id==ID_MC_TEMPERATURES_3){
            motortemp=(msg_rx.buf[4]+(msg_rx.buf[5]*256))/10;
            // Serial.printf("motortemp: %d\n",motortemp);
        }
        if(msg_rx.id==ID_MC_TEMPERATURES_1){
            invertertemp=(msg_rx.buf[0]+(msg_rx.buf[1]*256))/10;
            // Serial.printf("invtemp: %d\n",invertertemp);
        }
        if(msg_rx.id==ID_MC_FAULT_CODES){
            invfaults="";
            for(int i = 0; i<msg_rx.len;i++){
                uint8_t temp = msg_rx.buf[i];
                String tempo = String(temp,HEX);
                invfaults+=tempo;
            }
        }
        if(msg_rx.id==ID_VCU_PEDAL_READINGS){
            accel1_=(msg_rx.buf[0]+(msg_rx.buf[1]*256));
            accel2_=(msg_rx.buf[2]+(msg_rx.buf[3]*256));
            brake1_=(msg_rx.buf[4]+(msg_rx.buf[5]*256));
        }
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


    if(LoraTimer.check() && transmittedFlag){
        enableInterrupt=false;
        transmittedFlag=false;
        float starttime = millis();
        output="";
        output += packvoltage;
        output+=",";
        output+=motortemp;
        output+=",";
        output+=invertertemp;
        output+=",";
        output+=invfaults;
        output+=",";
        output+=accel1_;
        output+=",";
        output+=accel2_;
        output+=",";
        output+=brake1_;
        output+=",";
        output+=invertercurrent;
        output+=",";
        output+=torquereq;
        output+=",";
        output+=motorrpm;
        if (transmissionState == RADIOLIB_ERR_NONE) {
        // the packet was successfully transmitted
        Serial.println(F(" success!"));

        // print measured data rate
        Serial.print(F("[SX1276] Datarate:\t"));
        Serial.print(radio.getDataRate());
        Serial.println(F(" bps"));
        } else {
        // some other error occurred
        Serial.print(F("failed, code "));
        Serial.println(transmissionState);
        }        
        transmissionState = radio.startTransmit(output);
        Serial.printf("timestamp %f\n",starttime/1000);
        enableInterrupt=true;
    }
}
void parse_can_message() {
    while (CAN.read(msg_rx)) {

        write_to_SD(&msg_rx); // Write to SD card buffer (if the buffer fills up, triggering a flush to disk, this will take 8ms)
        
    }
}
void write_to_SD(CAN_message_t *msg) { // Note: This function does not flush data to disk! It will happen when the buffer fills or when the above flush timer fires
    digitalWrite(LED_BUILTIN,HIGH);
    // Calculate Time
    //This block is verified to loop through
    Serial.println("writing to SD buffer");
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