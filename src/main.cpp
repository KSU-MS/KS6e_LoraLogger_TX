/*
 * Teensy 4.1 crude logger
 yoinked from hytech racing "telemetry control unit" repo
 removed all features except can logging to SD
 and then removed LoRa specific code
 */
#include "core_pins.h"
#include <Arduino.h>
#include <SD.h>
#include <Wire.h>
#include <TimeLib.h>
#include <Metro.h>
#include <FlexCAN_T4.h>
// int pin_cs = 10;
// int pin_dio0 = 6;
// int pin_nrst = 7;
// int pin_dio1 = 5;
// // SX1276 radio = new Module(pin_cs, pin_dio0, pin_nrst, pin_dio1);
// String packVoltage="";
// String inverterTemp="";
/*
 * CAN Variables
 */
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> CAN;
static CAN_message_t msg_rx;
static CAN_message_t msg_tx;
// File/Buffer thingy
File logger;
/*
 * Variables to help with time calculation
 */
uint64_t global_ms_offset = 0;
uint64_t last_sec_epoch;
Metro timer_debug_RTC = Metro(1000);
Metro timer_flush = Metro(50);
void parse_can_message();
void write_to_SD(CAN_message_t *msg);
time_t getTeensy3Time();
void sd_date_time(uint16_t* date, uint16_t* time);
String date_time(int time);
void setup() {
    //SD logging init stuff
    delay(500); // Prevents suprious text files when turning the car on and off rapidly
    /* Set up Serial, CAN */
    Serial.begin(115200);

    /* Set up real-time clock */
    // get the time from here: https://www.epochconverter.com/?source=searchbar&q=time+v
    // Teensy3Clock.set(1704246446); // set time (epoch) at powerup  (COMMENT OUT THIS LINE AND PUSH ONCE RTC HAS BEEN SET!!!!)
    setSyncProvider(getTeensy3Time); // registers Teensy RTC as system time
  if (timeStatus() != timeSet) {
    Serial.println("RTC not set up - uncomment the Teensy3Clock.set() function call to set the time");
  } else {
    Serial.println("System time set to RTC");
      }
  last_sec_epoch = Teensy3Clock.get();

  FLEXCAN1_MCR &= 0xFFFDFFFF; // Enables CAN message self-reception
  CAN.begin();
  CAN.setBaudRate(500000);
  /* Set up SD card */
  Serial.println("Initializing SD card...");
  SdFile::dateTimeCallback(sd_date_time); // Set date/time callback function
  if (!SD.begin(BUILTIN_SDCARD)) { // Begin Arduino SD API (Teensy 3.5)
    Serial.println("SD card failed or not present");
  }
  // Make name of current time
  const char *filename = date_time(Teensy3Clock.get()).c_str();
  if (SD.exists(filename)) { // Print error if name is taken
    Serial.println("You generated a duplicate file name... Go check RTC.");
  }
  else if (!SD.exists(filename)) { // Open file for writing
    logger = SD.open(filename, (uint8_t)O_WRITE | (uint8_t)O_CREAT);
  }

  if (logger) { // Print on open
    Serial.print("Successfully opened SD file: ");
    Serial.println(filename);
  } else { // Print on fail
    Serial.println("Failed to open SD file");
  }

  logger.println("time,msg.id,msg.len,data"); // Print CSV heading to the logfile
  logger.flush();
}
void loop() {
    /* Process and log incoming CAN messages */
  parse_can_message();
/* Flush data to SD card occasionally */
  if (timer_flush.check()) {
    logger.flush(); // Flush data to disk (data is also flushed whenever the 512 Byte buffer fills up, but this call ensures we don't lose more than a second of data when the car turns off)
  }
/* Print timestamp to serial occasionally */
  if (timer_debug_RTC.check()) {
    unsigned long current_timestamp = Teensy3Clock.get();
    Serial.println(current_timestamp);
    msg_tx.id=0x3FF;
    memcpy(&msg_tx.buf[0], &current_timestamp, sizeof(current_timestamp));
    CAN.write(msg_tx);
  }

}
void parse_can_message() {
  while (CAN.read(msg_rx)) {
    
        write_to_SD(&msg_rx); // Write to SD card buffer (if the buffer fills up, triggering a flush to disk, this will take 8ms)
        
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
  digitalToggle(13);
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

//input epoch time, return human-readable string formatted time
String date_time(int time) {
  String minutes = "";
  if (minute(time) > 10){
    minutes = String(minute(time));
  }else{
    minutes = "0"+String(minute(time));
  }
  String seconds = "";
  if (second(time) > 10){
    seconds = String(second(time));
  }else{
    seconds = "0"+String(second(time));
  }
  String outString = "MDY_" + String(month(time)) + "-" + String(day(time)) +
                     "-" + String(year(time)) + "_HMS_" + String(hour(time)) +
                     "-" + minutes + "-" + seconds +
                     ".CSV";

  return outString;
}