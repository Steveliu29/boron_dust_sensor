/*
    This is the MIT License for the demo of HM3301 Dust Sensor.
    basic_demo.ino
    Example for Seeed PM2.5 Sensor(HM300)

    Copyright (c) 2018 Seeed Technology Co., Ltd.
    Website    : www.seeed.cc
    Author     : downey
    Create Time: August 2018
    Change Log :

    The MIT License (MIT)

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to deal
    in the Software without restriction, including without limitation the rights
    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in
    all copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
    THE SOFTWARE.
*/

#include <Seeed_HM330X.h>
#include <vector>

// #define SERIAL_OUTPUT_ENABLE
#define TIME_SYNC_ENABLE
#define SLEEP_ENABLE

// SLEEP TIME = 30min * 60sec/min * 1000ms/sec 
#define SLEEP_TIME 30 * 60 * 1000
#define ONE_DAY_MILLIS (24 * 60 * 60 * 1000)
#define MAX_SAMPLE 48
// This is the TIME_ZONE for EST, change it to corresponding time zone when shipped to Kenya 
#define TIME_ZONE -5 
// unsigned long lastSync = millis();

uint16_t sample_counter = 0;
uint16_t sample_sent = 0;
system_tick_t sleep_time; 
const int DUST_SENSOR = D6;

HM330X sensor;
uint8_t buf[30];

// before setup - enable feature
STARTUP(System.enableFeature(FEATURE_RESET_INFO));
// Enable logging as we ware looking at messages that will be off-line - need to connect to serial terminal
SerialLogHandler logHandler(LOG_LEVEL_INFO);

typedef struct data {
    uint16_t year = 0;
    uint16_t month = 0;
    uint16_t day = 0;
    uint16_t hour = 0;
    uint16_t minute = 0;
    uint16_t ae_value [3] = {0,0,0};
} Data;


std::vector<Data> sensor_record;

const char* str[] = {
                     "PM1.0 concentration(Atmospheric environment,unit:ug/m3): ",
                     "PM2.5 concentration(Atmospheric environment,unit:ug/m3): ",
                     "PM10 concentration(Atmospheric environment,unit:ug/m3): ",
                    };


HM330XErrorCode publish_daily_record() {
    while(!sensor_record.empty()){
        Data data_to_publish = sensor_record.front();
        
        // Some String manipulation
        String to_publish = "[";
        to_publish.concat((String)data_to_publish.year);
        to_publish.concat(",");
        to_publish.concat((String)data_to_publish.month);
        to_publish.concat(",");
        to_publish.concat((String)data_to_publish.day);
        to_publish.concat(",");
        to_publish.concat((String)data_to_publish.hour);
        to_publish.concat(",");
        to_publish.concat((String)data_to_publish.minute);
        to_publish.concat(",");
        to_publish.concat((String)data_to_publish.ae_value[0]);
        to_publish.concat(",");
        to_publish.concat((String)data_to_publish.ae_value[1]);
        to_publish.concat(",");
        to_publish.concat((String)data_to_publish.ae_value[2]);
        to_publish.concat("]");
        
        safeDelay(1000);

        // Check for connection
        // if YES, transmitted everything in the buffer
        // if NO, leave the current data in the buffer, send it next day
        if (Particle.connected()){
            Particle.publish("Dust_Reading", to_publish);
            sensor_record.erase(sensor_record.begin());
            sample_sent = sample_sent + 1;
        } else {
            break;
        }
    }

    return NO_ERR;
}


/*parse buf with 29 uint8_t-data*/
HM330XErrorCode parse_result(uint8_t* data, Data& new_sample) {
    uint16_t value = 0;
    if (NULL == data) {
        return ERROR_PARAM;
    }
    
    // my_record.sensor_no = (uint16_t) data[2] << 8 | data[5];

    for (int i = 5; i < 8; i++) {
        value = (uint16_t) data[i * 2] << 8 | data[i * 2 + 1];

        new_sample.ae_value[i - 5] = new_sample.ae_value[i - 5] + value;
               
    }

    return NO_ERR;
}


HM330XErrorCode print_result(const char* str, uint16_t value) {
    if (NULL == str) {
        return ERROR_PARAM;
    }
    Log.info(str + String(value));

    return NO_ERR;
}


void safeDelay(unsigned long delayMillis) { // Ensures Particle functions operate during long delays
   unsigned long timeStamp = millis();
   while (millis() - timeStamp < delayMillis) {
      delay(10);
      Particle.process();
   }
}

SYSTEM_MODE(SEMI_AUTOMATIC);
SYSTEM_THREAD(ENABLED);

void setup() {

    // Synchronize the time during setup
    Particle.connect();
    if (waitFor(Particle.connected, 180000)){
        Particle.syncTime();
        waitUntil(Particle.syncTimeDone);
    } else {
        // No Cellular found, sleep and reset
        SystemSleepConfiguration config;
        config.mode(SystemSleepMode::ULTRA_LOW_POWER)
              .duration(30min);
        System.sleep(config);

        System.reset();
    }
    
    // pinMode(DUST_SENSOR, OUTPUT);
    // digitalWrite(DUST_SENSOR, HIGH);
    
    // safeDelay(10000);

    // DEBUG USE: Log the previous reset reason
    if (System.resetReason() == RESET_REASON_PIN_RESET) {
        Log.info("Restarted due to a pin reset");
    } else if (System.resetReason() == RESET_REASON_USER) { // Check to see if we are starting from a pin reset or a reset in the sketch
        Log.info("Restarted due to a user reset");
    } else Log.info("System reset reason %i", System.resetReason());

    Time.zone(TIME_ZONE); // Adjust for the time zone

    if (sensor.init()) {
        Log.info("Init Failed");
        Particle.disconnect();
        Cellular.off();
        // Trap if init failed
        // Init Failure usually caused by bad physical connection to the HM3301 dust sensor
        while (1){
            Particle.process(); 
        };
    } else {
        Log.info("Init Success");
        Particle.disconnect();
        Cellular.off();
    }

}


void loop() {  
    
    Data new_sample;

    // Take 5 samples with 1s interval 
    Log.info("Taking samples");
    for (int i = 0; i < 5; i++){
		if (sensor.read_sensor_value(buf, 29)) {
        	Log.info("Read Result Failed");
    	}
        parse_result(buf, new_sample);
		safeDelay(1000);
    }

    // Average 5 samples
	for (int i = 0; i < 3; i++){
		new_sample.ae_value[i] = new_sample.ae_value[i] / 5;
    }

    // Record the time
    new_sample.year = Time.year();
    new_sample.month = Time.month();
    new_sample.day = Time.day();
    new_sample.hour = Time.hour();
    new_sample.minute = Time.minute();
    
    // Add it to the sensor record and increment the sampler counter
    sensor_record.emplace_back(new_sample);
    sample_counter = sample_counter + 1;

    // Sleep time = 30min - 5s sample taking time
    sleep_time = SLEEP_TIME - 5000; // 1800000ms = 1800s = 30min

    // For the second last sample taken, open up the connection with cloud
    if (sample_counter == MAX_SAMPLE - 1){
        Log.info("Attemp to connect");
        Particle.connect();

        int i = 0;
        for (; i < 90; i++){
            Particle.process();
            safeDelay(2000);
            if (Particle.connected()){
                Log.info("Connected to the Cloud");

                // Inform the cloud about the availability of OTA
                Particle.publish("HELLO ICON LAB"); 
                safeDelay(2000);
                sleep_time = sleep_time - 2000;
                break;
            }
        }

        if (i == 90){
            Log.info("Fail to connect to the cloud, abort.");
            Particle.disconnect();
            Cellular.off();
            waitFor(Cellular.isOff, 60000);
        }

        sleep_time = sleep_time - (i+1)*2000; // offset the sleep time

#ifdef TIME_SYNC_ENABLE
    // Code taken from Particle's tutorial
    // https://docs.particle.io/cards/firmware/cloud-functions/particle-synctime/
    //
    if (Particle.connected()) {
        // Request time synchronization from the Particle Device Cloud
        Particle.syncTime();
        safeDelay(2000);
        sleep_time = sleep_time - 2000;
    }
    
#endif

    }

    // Publish the record to the cloud every 48 samples (48 samples * 30 min/sample = 24 hr)
    // Reset all the counters
    else if (sample_counter == MAX_SAMPLE){    
        Log.info("Ready to publish");
        //Particle.publish("Total Sample: " + String(sample_counter));      
        publish_daily_record();
        sample_counter = 0;
        safeDelay(2000); // Give the device some time to process all the Particle.publish()
        if (Particle.connected()){
            Particle.disconnect();
            Cellular.off();
            waitFor(Cellular.isOff, 60000);
        }
        Log.info("Switch off network facility");
        sleep_time = sleep_time - 2000 - sample_sent * 1000; // offset the sleep time
        sample_sent = 0;
    }

    // Log.info("Turning off the sensor");
    // Wire.end();
    // digitalWrite(DUST_SENSOR, LOW);

    // if (Particle.connected())
    // {
    //     Particle.publish("Start Sleeping");
    // }

    Log.info("Start Sleeping");
#ifdef SLEEP_ENABLE
    // Last cycle will have network enabled for potential OTA
    if (sample_counter == MAX_SAMPLE - 1){
        SystemSleepConfiguration network_config;
        network_config.mode(SystemSleepMode::STOP)
              .network(NETWORK_INTERFACE_CELLULAR)
              .duration(sleep_time);
        System.sleep(network_config);
    } else {
        SystemSleepConfiguration regular_config;
        regular_config.mode(SystemSleepMode::STOP)
              .duration(sleep_time);
        System.sleep(regular_config);
    }
#else
    safeDelay(sleep_time);
#endif
    // if (Particle.connected())
    // {
    //     Particle.publish("Waked Up from Sleep");
    // }
    // Log.info("Turning on the sensor");
    // digitalWrite(DUST_SENSOR, HIGH);
    // Log.info("Waiting for sensor starting up...");
    // Wire.begin();
    // safeDelay(30000);

    Log.info("Counter number is " + String(sample_counter));
}
