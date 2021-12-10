/******************************************************/
//       THIS IS A GENERATED FILE - DO NOT EDIT       //
/******************************************************/

#include "Particle.h"
#line 1 "c:/Users/10336/OneDrive/Particle/Working_Dust_Sensor/src/Working_Dust_Sensor.ino"
/*
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

HM330XErrorCode record_reset();
HM330XErrorCode publish_daily_record();
HM330XErrorCode parse_result(uint8_t* data);
HM330XErrorCode parse_result_value(uint8_t* data);
HM330XErrorCode print_result(const char* str, uint16_t value);
void setup();
void loop();
#line 34 "c:/Users/10336/OneDrive/Particle/Working_Dust_Sensor/src/Working_Dust_Sensor.ino"
#ifdef  ARDUINO_SAMD_VARIANT_COMPLIANCE
    #define SERIAL_OUTPUT SerialUSB
#else
    #define SERIAL_OUTPUT Serial
#endif

// #define SERIAL_OUTPUT_ENABLE
// #define TIME_SYNC_ENABLE
// #define SLEEP_ENABLE

#define SLEEP_TIME 15min
#define ONE_DAY_MILLIS (24 * 60 * 60 * 1000)
#define MAX_SAMPLE 48 
unsigned long lastSync = millis();

uint8_t sample_counter = 0;

HM330X sensor;
uint8_t buf[30];

typedef struct dust_record {
    uint16_t sensor_no;
    uint16_t time [MAX_SAMPLE];
    uint16_t ae_value [MAX_SAMPLE][3];
    // uint16_t spm_value [3];

} Dust_record;

Dust_record my_record;
// Remember to push the collected value within 24hrs all at once

const char* str[] = {"sensor num: ", "PM1.0 concentration(CF=1,Standard particulate matter,unit:ug/m3): ",
                     "PM2.5 concentration(CF=1,Standard particulate matter,unit:ug/m3): ",
                     "PM10 concentration(CF=1,Standard particulate matter,unit:ug/m3): ",
                     "PM1.0 concentration(Atmospheric environment,unit:ug/m3): ",
                     "PM2.5 concentration(Atmospheric environment,unit:ug/m3): ",
                     "PM10 concentration(Atmospheric environment,unit:ug/m3): ",
                    };

HM330XErrorCode record_reset() {
    int size = (sizeof(uint16_t));
    memset(my_record.time, 0, MAX_SAMPLE * size);
    memset(my_record.ae_value, 0, MAX_SAMPLE * size * 3);

    return NO_ERR;
}

HM330XErrorCode publish_daily_record() {
    for (int i = 0; i < MAX_SAMPLE; i++){
        String to_publish = (String) my_record.sensor_no;
        to_publish.concat(",");
        to_publish.concat((String)my_record.time[i]);
        to_publish.concat(",");
        to_publish.concat((String)my_record.ae_value[i][0]);
        to_publish.concat(",");
        to_publish.concat((String)my_record.ae_value[i][1]);
        to_publish.concat(",");
        to_publish.concat((String)my_record.ae_value[i][2]);
        
        delay(1000);
        Particle.publish(to_publish);
    }

    return NO_ERR;
}


/*parse buf with 29 uint8_t-data*/
HM330XErrorCode parse_result(uint8_t* data) {
    uint16_t value = 0;
    if (NULL == data) {
        return ERROR_PARAM;
    }
    
    my_record.sensor_no = (uint16_t) data[2] << 8 | data[5];

    for (int i = 2; i < 8; i++) {
        value = (uint16_t) data[i * 2] << 8 | data[i * 2 + 1];

        if (i >= 5){
           my_record.ae_value[sample_counter][i - 5] = my_record.ae_value[sample_counter][i - 5] + value;
        } 
        
    }

    return NO_ERR;
}

#ifdef SERIAL_OUTPUT_ENABLE

HM330XErrorCode parse_result_value(uint8_t* data) {
    if (NULL == data) {
        return ERROR_PARAM;
    }
    for (int i = 0; i < 28; i++) {
        SERIAL_OUTPUT.print(data[i], HEX);
        SERIAL_OUTPUT.print("  ");
        if ((0 == (i) % 5) || (0 == i)) {
            SERIAL_OUTPUT.println("");
        }
    }
    uint8_t sum = 0;
    for (int i = 0; i < 28; i++) {
        sum += data[i];
    }
    if (sum != data[28]) {
        SERIAL_OUTPUT.println("wrong checkSum!!!!");
    }
    SERIAL_OUTPUT.println("");
    return NO_ERR;
}

HM330XErrorCode print_result(const char* str, uint16_t value) {
    if (NULL == str) {
        return ERROR_PARAM;
    }
    SERIAL_OUTPUT.print(str);
    SERIAL_OUTPUT.println(value);

    return NO_ERR;
}

#endif


/*30s*/
SYSTEM_MODE(AUTOMATIC);
// SYSTEM_THREAD(ENABLED);

void setup() {
    record_reset();

#ifdef SERIAL_OUTPUT_ENABLE
    SERIAL_OUTPUT.begin(115200);
    delay(100);
    SERIAL_OUTPUT.println("Serial start");
    if (sensor.init()) {
        SERIAL_OUTPUT.println("HM330X init failed!!!");
        while (1);
    }

#else
    if (sensor.init()) {
        Particle.publish("Init Failed");
        while (1);
    } else {
        Particle.publish("Init Success");
    }
#endif

}


void loop() {  

#ifdef SERIAL_OUTPUT_ENABLE
    for (int i = 0; i < 5; i++){
		if (sensor.read_sensor_value(buf, 29)) {
        	SERIAL_OUTPUT.println("HM330X read result failed!!!");
    	}
        parse_result_value(buf);
        parse_result(buf);
		delay(1000);
    }

	for (int i = 0; i < 3; i++){
		my_record.ae_value[sample_counter][i] = my_record.ae_value[sample_counter][i] / 5;
    }

    SERIAL_OUTPUT.println("");
    delay(5000);

	// Print result out to the serial line
    print_result(str[0], my_record.sensor_no);
	for (int i = 4; i < 7; i++) {
		print_result(str[i], my_record.ae_value[sample_counter][i - 4]);
	}
    SERIAL_OUTPUT.println("");

#else

    for (int i = 0; i < 5; i++){
		if (sensor.read_sensor_value(buf, 29)) {
        	Particle.publish("Read Result Failed");
    	}
        parse_result(buf);
		delay(1000);
    }

	for (int i = 0; i < 3; i++){
		my_record.ae_value[sample_counter][i] = my_record.ae_value[sample_counter][i] / 5;
    }

    delay(5000);

#endif

#ifdef TIME_SYNC_ENABLE
    // Code taken from Particle's tutorial
    // https://docs.particle.io/cards/firmware/cloud-functions/particle-synctime/
    //
    if (millis() - lastSync > ONE_DAY_MILLIS) {
        // Request time synchronization from the Particle Device Cloud
        Particle.syncTime();
        lastSync = millis();
    }
#endif
    
    my_record.time[sample_counter] = Time.minute();

    sample_counter = sample_counter + 1;

    if (sample_counter == MAX_SAMPLE){

        publish_daily_record();
        sample_counter = 0;
        record_reset();
    }

#ifdef SLEEP_ENABLE
    SystemSleepConfiguration config;
    config.mode(SystemSleepMode::ULTRA_LOW_POWER)
          // .network(NETWORK_INTERFACE_CELLULAR)
          .duration(SLEEP_TIME);
    System.sleep(config);
#else
    delay(15000);
#endif

    while (!Particle.connected()){
        delay(5000);
    }
    // Particle.publish(String(sample_counter));
    // Particle.connect();
    // while (!Particle.connected()){
    //     delay(5000);
    // }
}
