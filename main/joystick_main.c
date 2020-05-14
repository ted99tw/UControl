// Copyright 2019 Mark Wolfe.

// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     https://www.apache.org/licenses/LICENSE-2.0

// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event_loop.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_bt.h"

#include "esp_hidd_prf_api.h"
#include "esp_bt_defs.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_gatt_defs.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "driver/gpio.h"
#include "hid_dev.h"
#include "driver/touch_pad.h"
#include "driver/adc.h"

#include "joystick_buttons.h"
#include "mpu6050_raw.c"
#define HID_JOYSTICK_TAG "HID_JOYSTICK"
#define BUILT_IN_LED 22

static uint16_t hid_conn_id = 0;
static bool sec_conn = false;

bool thispunch = false;  
uint32_t punches = 0;
bool CALIB = true;      // self calibrate at starting

// if GAP between 2 pos too many, treat as noise 
uint8_t prev1y=127, prev1z=127, prev2y=127, prev2z=127, GAP=30;     
static bool connected = false;
#define CHAR_DECLARATION_SIZE   (sizeof(uint8_t))
static void hidd_event_callback(esp_hidd_cb_event_t event, esp_hidd_cb_param_t *param);
#define HIDD_DEVICE_NAME            "Body Gamepad C"
//#define VDIRECTION 					(1) 	// 0=H , 1=V
static uint8_t hidd_service_uuid128[] = {
    /* LSB <--------------------------------------------------------------------------------> MSB */
    //first uuid, 16bit, [12],[13] is the value
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0x12, 0x18, 0x00, 0x00,
};

static esp_ble_adv_data_t hidd_adv_data = {
    .set_scan_rsp = false,
    .include_name = true,
    .include_txpower = true,
    .min_interval = 0x0006, //slave connection min interval, Time = min_interval * 1.25 msec
    .max_interval = 0x0010, //slave connection max interval, Time = max_interval * 1.25 msec
    .appearance = 0x03c0,       //HID Generic,
    .manufacturer_len = 0,
    .p_manufacturer_data =  NULL,
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = sizeof(hidd_service_uuid128),
    .p_service_uuid = hidd_service_uuid128,
    .flag = 0x6,
};

static esp_ble_adv_params_t hidd_adv_params = {
    .adv_int_min        = 0x20,
    .adv_int_max        = 0x30,
    .adv_type           = ADV_TYPE_IND,
    .own_addr_type      = BLE_ADDR_TYPE_PUBLIC,
    //.peer_addr            =
    //.peer_addr_type       =
    .channel_map        = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

static void hidd_event_callback(esp_hidd_cb_event_t event, esp_hidd_cb_param_t *param)
{
    switch(event) {
    case ESP_HIDD_EVENT_REG_FINISH: {
        if (param->init_finish.state == ESP_HIDD_INIT_OK) {
            //esp_bd_addr_t rand_addr = {0x04,0x11,0x11,0x11,0x11,0x05};
            esp_ble_gap_set_device_name(HIDD_DEVICE_NAME);
            esp_ble_gap_config_adv_data(&hidd_adv_data);
        }
        break;
    }
    case ESP_BAT_EVENT_REG: {
        break;
    }
    case ESP_HIDD_EVENT_DEINIT_FINISH:
        break;
    case ESP_HIDD_EVENT_BLE_CONNECT: {
        ESP_LOGI(HID_JOYSTICK_TAG, "ESP_HIDD_EVENT_BLE_CONNECT");
        connected = true;
        hid_conn_id = param->connect.conn_id;
        break;
    }
    case ESP_HIDD_EVENT_BLE_DISCONNECT: {
        sec_conn = false;
        connected = false;
        ESP_LOGI(HID_JOYSTICK_TAG, "ESP_HIDD_EVENT_BLE_DISCONNECT");
        esp_ble_gap_start_advertising(&hidd_adv_params);
        break;
    }
    case ESP_HIDD_EVENT_BLE_VENDOR_REPORT_WRITE_EVT: {
        ESP_LOGI(HID_JOYSTICK_TAG, "%s, ESP_HIDD_EVENT_BLE_VENDOR_REPORT_WRITE_EVT", __func__);
        ESP_LOG_BUFFER_HEX(HID_JOYSTICK_TAG, param->vendor_write.data, param->vendor_write.length);
    }
    default:
        break;
    }
    return;
}

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event) {
    case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
        esp_ble_gap_start_advertising(&hidd_adv_params);
        break;
    case ESP_GAP_BLE_SEC_REQ_EVT:
        for(int i = 0; i < ESP_BD_ADDR_LEN; i++) {
            ESP_LOGD(HID_JOYSTICK_TAG, "%x:",param->ble_security.ble_req.bd_addr[i]);
        }
        esp_ble_gap_security_rsp(param->ble_security.ble_req.bd_addr, true);
        break;
    case ESP_GAP_BLE_AUTH_CMPL_EVT:
        sec_conn = true;
        esp_bd_addr_t bd_addr;
        memcpy(bd_addr, param->ble_security.auth_cmpl.bd_addr, sizeof(esp_bd_addr_t));
        ESP_LOGI(HID_JOYSTICK_TAG, "remote BD_ADDR: %08x%04x",\
                (bd_addr[0] << 24) + (bd_addr[1] << 16) + (bd_addr[2] << 8) + bd_addr[3],
                (bd_addr[4] << 8) + bd_addr[5]);
        ESP_LOGI(HID_JOYSTICK_TAG, "address type = %d", param->ble_security.auth_cmpl.addr_type);
        ESP_LOGI(HID_JOYSTICK_TAG, "pair status = %s",param->ble_security.auth_cmpl.success ? "success" : "fail");
        if(!param->ble_security.auth_cmpl.success) {
            ESP_LOGE(HID_JOYSTICK_TAG, "fail reason = 0x%x",param->ble_security.auth_cmpl.fail_reason);
        }
        break;
    default:
        break;
    }
}

static uint8_t readJoystickChannel(adc1_channel_t channel)
{
    adc1_config_width(ADC_WIDTH_BIT_10);                  //Range 0-1023
    adc1_config_channel_atten(channel, ADC_ATTEN_DB_11);  //ADC_ATTEN_DB_11 = 0-3,6V
    return (uint8_t)(adc1_get_raw(channel) >> 2);         //Read analog and shift to 0-255
}

void checkPunch(int16_t x, int16_t y, int16_t z){
    if (y==32767 || y==-32768 || z==32767 || z==-32768){
        thispunch = true;
        //printf( "punch ..................................... punch ............!!! \n");
    } else {
        thispunch = false;
    }
}

// 過濾雜訊，如果超過GAP值可能為雜訊
bool goodSignal(uint8_t z, uint8_t y){
    if (abs(prev1z-z) <= GAP && abs(prev1y-y) <= GAP){
        //新點離前一點近
        prev2z = prev1z;
        prev2y = prev1y;
        prev1z = z;
        prev1y = y;
        return true;
    } else if (abs(prev2z-z) <= GAP && abs(prev2y-y) <= GAP){
        //新點離前二點近，則前一點為雜訊要去掉
        prev1z = z;
        prev1y = y;
        return true;
    } else {
        //未靠近任何一點，本身可能是雜訊，先記下來
        prev2z = prev1z;
        prev2y = prev1y;
        prev1z = z;
        prev1y = y;
        return false;
    } 
}

// 縮小震動，類似開根號乘以10
uint8_t smooth(uint8_t v){
    int temp1 = (v-127)*(v-127)/127;
    uint8_t temp2 = v-127 > 0 ? temp1 : -temp1;
    return temp2+127; 
}

uint32_t getAcce(){
    //uint16_t result; 
    int16_t x, y, z;
    //int ret;
    uint8_t* data_rd = (uint8_t*) malloc(BUFF_SIZE);
    //ret = mpu6050_read_acce(I2C_MASTER_NUM, data_rd, BUFF_SIZE);
    mpu6050_read_acce(I2C_MASTER_NUM, data_rd, BUFF_SIZE);
    //disp_buf(data_rd, BUFF_SIZE);
    x = (data_rd[0] << 8) + data_rd[1];   // x : -16384 ~ 16384
    y = (data_rd[2] << 8) + data_rd[3];
    z = (data_rd[4] << 8) + data_rd[5];

    checkPunch(x,y,z);                      // if x,y,z too wide, treated as punch and 

    x += MAXVALUE;                      // + 16384
    x = x >= 0 ? x : 0;    
    y += MAXVALUE;
    y = y >= 0 ? y : 0;  
    z += MAXVALUE;
    z = z >= 0 ? z : 0;
    //printf("mpu6050BBB... xyz = %d %d %d\n", x,y,z);
    x = 255 * x / MAXVALUE / 2;         // x : 0~255    
    x = x < 256 ? x : 255 ; 
    y = 255 * y / MAXVALUE / 2;         // y : 0~255
    y = y < 256 ? y : 255 ; 
    z = 255 * z / MAXVALUE / 2;         // y : 0~255
    z = z < 256 ? z : 255 ; 
    //printf("mpu6050... xyz = %d %d %d\n", x,y,z);
    return ((x & 0xFF)<<16) + ((y & 0xFF)<<8) + (z & 0xFF);
}

static void read_joystick_task(void *pvParameter)
{
    uint8_t js1x,js1y,js1z,js2x=127,js2y=127;
    uint32_t current_sum;
    uint8_t dir = 1;     	//0:平面前 +x/+y, 1:小直立(0逆90度) +z/+y, 2:平面大直立() +x/+z 
    uint16_t buttons = 0;
    uint32_t last_sum = 0;
    uint16_t last_buttons = 0;
    uint8_t dx=0, dy=0, dz=0;
    uint16_t touch_value, touch5, touch6, touch8, touch9;
    uint16_t Delta = 300;
    bool lastpunch = false;
    float rate = 0.0, pos = 0.0;
    uint32_t id = 0;
    portTickType starttime, lasttime, temptime;

    if (CALIB){
        //calibrate x,y 
        vTaskDelay( 1000 / portTICK_PERIOD_MS );
        int x1=0, y1=0, z1=0;
        for (int i = 0; i < 16; i++){
            uint32_t r = getAcce();    // uint32: 0x00xxyyzz
            x1 += (r & 0xff0000) >> 16;
            y1 += (r & 0xff00)  >> 8;
            z1 += (r & 0xff);
        }
        dx = (x1 >> 4) & 0xff; 
        dy = (y1 >> 4) & 0xff; 
        dz = (z1 >> 4) & 0xff; 
        dx -= 127;
        dy -= 127;
        dz -= 127;   
        printf("x y z = %d %d %d \n",dx, dy, dz );
    }

    //turn on led after calibration
    gpio_set_level(BUILT_IN_LED, 0);

    //calibrate touchPad A and B
    touch_pad_read_raw_data(5, &touch5);
    touch_pad_read_raw_data(6, &touch6);
    touch_pad_read_raw_data(8, &touch8);
    touch_pad_read_raw_data(9, &touch9);

	starttime = xTaskGetTickCount();
	lasttime = starttime;
    while(true) {
		//mpu6050 acceleration
        uint32_t r = getAcce(); // and check thispunch
        js1x = (r & 0xff0000) >> 16; 
        js1y = (r & 0xff00) >> 8 ;
        js1z = (r & 0xff);

        if (CALIB){
            js1x -= dx;         
            js1y -= dy;
            js1z -= dz;
        }

        // 縮小震動，類似開根號乘以10
        printf( "org %d %d %d --- ", js1x, js1y, js1z);
        js1x = smooth(js1x);
        js1y = smooth(js1y);
        js1z = smooth(js1z);
        printf( "%d %d %d\n", js1x, js1y, js1z);

        touch_pad_read_raw_data(9, &touch_value);
        if (touch_value < touch9 - Delta) buttons |= (1<<9);
        else buttons &= ~(1<<9);
        printf( "touchpad9 = %d \n", touch_value);
	
		touch_pad_read_raw_data(8, &touch_value);
		if (touch_value < touch8 - Delta) buttons |= (1<<8);
	    else buttons &= ~(1<<8);
        printf( "touchpad8 = %d \n", touch_value);

        //select/coin button
        touch_pad_read_raw_data(5, &touch_value);
        if (touch_value < touch5 - Delta) buttons |= (1<<5);
        else buttons &= ~(1<<5);
        printf( "touchpad5 = %d \n", touch_value);

        //start button
        touch_pad_read_raw_data(6, &touch_value);
        if (touch_value < touch6 - Delta) buttons |= (1<<6);
        else buttons &= ~(1<<6);
        printf( "touchpad6 = %d \n", touch_value);

        //punch, if acc too much, treated as punch , then activate button 6 as well
        int TH = 100;
        id++;
        
        // treated as punch if not close to previous punch
        if (xTaskGetTickCount()-lasttime > 8){
            if (thispunch){
                buttons |= (1<<9);
                lastpunch = true;
                lasttime = xTaskGetTickCount();
            }
        } else {	
            // 2nd punch treated as non-punch if too close between 2 punches            
	        if (lastpunch && thispunch) {          // 太靠近不算，跳過一次
	        	lastpunch = false;
	        	buttons &= ~(1<<9);
	        } else if (thispunch && !lastpunch){   // 跳過一次就行了
	        	buttons |= (1<<9);
	        	lastpunch = true;
	        }
		} 
        if (((buttons >> 9) & 1) == 1) {
        	printf( "punch .............................. punch %d!!! \n", ++punches);
		}

        int TH2 = 20;
        if (dir == 0){
		    if (abs(js1x - 127) + abs(js1y - 127) > TH2){		//
	            //ESP_LOGI(HID_JOYSTICK_TAG, "buttons.. %d JS1 X=%d Y=%d Z=%d JS2 X=%d Y=%d", buttons, js1x, js1y, js1z, js2x, js2y);
	    		esp_hidd_send_joystick_value(hid_conn_id, buttons, js1x, 255-js1y, js2x, js2y);
	    	}	
	    } else if (dir == 1){	
            //js1z = abs(js1z - 127) > TH2 ? js1z : 127;
            //js1y = abs(js1y - 127) > TH2 ? js1y : 127;
            if (goodSignal(js1z, js1y)){
                esp_hidd_send_joystick_value(hid_conn_id, buttons, js1z, js1y, js2x, js2y);
            }
            /*
	    	if (abs(js1z - 127) > TH2 || abs(js1y - 127) > TH2){
	            //ESP_LOGI(HID_JOYSTICK_TAG, "buttons.. %d JS1 X=%d Y=%d Z=%d JS2 X=%d Y=%d", buttons, js1z, js1y, js1z, js2x, js2y);
	            // V/H for Vertical/Horizontal	
	    		esp_hidd_send_joystick_value(hid_conn_id, buttons, js1z, js1y, js2x, js2y);
	    	} else {
                esp_hidd_send_joystick_value(hid_conn_id, buttons, 127, 127, js2x, js2y);
            }
            */
	    } else {
	    	if (abs(js1z - 127) + abs(js1x - 127) > TH2){	
	    		esp_hidd_send_joystick_value(hid_conn_id, buttons, js1x, 255-js1z, js2x, js2y);
	    	}
	    }	        

        vTaskDelay( 50 / portTICK_PERIOD_MS );
    }

}

static esp_err_t read_joystick_init(void)
{

    xTaskCreate(read_joystick_task, "read_joystick_task", 2048, NULL, 4, NULL);

    return ESP_OK;
}

void my_touch_pad_init(){
    // light up built_in_led pin 22
    gpio_pad_select_gpio(BUILT_IN_LED);
    gpio_set_direction(BUILT_IN_LED, GPIO_MODE_OUTPUT);
    gpio_set_level(BUILT_IN_LED, 1);

	// Initialize touch pad peripheral.
    // The default fsm mode is software trigger mode.
    touch_pad_init();
    // Set reference voltage for charging/discharging
    // In this case, the high reference valtage will be 2.7V - 1V = 1.7V
    // The low reference voltage will be 0.5
    // The larger the range, the larger the pulse count value.
    touch_pad_set_voltage(TOUCH_HVOLT_2V7, TOUCH_LVOLT_0V5, TOUCH_HVOLT_ATTEN_1V);
    touch_pad_config(5, 0); //gpio 12
    touch_pad_config(6, 0); //gpio 14
    touch_pad_config(8, 0); //gpio 33
    touch_pad_config(9, 0); //gpio 32
    /*
    for (int i = 0; i < 10; i++){
        touch_pad_config(i, 0);	// 5th pad no threshold
        //touch_pad_config(6, 0);
    }
    */
    touch_pad_filter_start(10);	//filtering per 10 ms
}

void app_main()
{
    esp_err_t ret;

    // init touch pad T5 and T6
    my_touch_pad_init();

    // Initialize NVS.
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret) {
        ESP_LOGE(HID_JOYSTICK_TAG, "%s initialize controller failed\n", __func__);
        return;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret) {
        ESP_LOGE(HID_JOYSTICK_TAG, "%s enable controller failed\n", __func__);
        return;
    }

    ret = esp_bluedroid_init();
    if (ret) {
        ESP_LOGE(HID_JOYSTICK_TAG, "%s init bluedroid failed\n", __func__);
        return;
    }

    ret = esp_bluedroid_enable();
    if (ret) {
        ESP_LOGE(HID_JOYSTICK_TAG, "%s init bluedroid failed\n", __func__);
        return;
    }

    if((ret = esp_hidd_profile_init()) != ESP_OK) {
        ESP_LOGE(HID_JOYSTICK_TAG, "%s init bluedroid failed\n", __func__);
    }

    ///register the callback function to the gap module
    esp_ble_gap_register_callback(gap_event_handler);
    esp_hidd_register_callbacks(hidd_event_callback);

    /* set the security iocap & auth_req & key size & init key response key parameters to the stack*/
    esp_ble_auth_req_t auth_req = ESP_LE_AUTH_BOND;     //bonding with peer device after authentication
    esp_ble_io_cap_t iocap = ESP_IO_CAP_NONE;           //set the IO capability to No output No input
    uint8_t key_size = 16;      //the key size should be 7~16 bytes
    uint8_t init_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
    uint8_t rsp_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
    esp_ble_gap_set_security_param(ESP_BLE_SM_AUTHEN_REQ_MODE, &auth_req, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_IOCAP_MODE, &iocap, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_MAX_KEY_SIZE, &key_size, sizeof(uint8_t));
    /* If your BLE device act as a Slave, the init_key means you hope which types of key of the master should distribute to you,
    and the response key means which key you can distribute to the Master;
    If your BLE device act as a master, the response key means you hope which types of key of the slave should distribute to you,
    and the init key means which key you can distribute to the slave. */
    esp_ble_gap_set_security_param(ESP_BLE_SM_SET_INIT_KEY, &init_key, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_SET_RSP_KEY, &rsp_key, sizeof(uint8_t));

    //init mpu6050
    mpu6050_myinit(); 
    /*
    if((ret = read_mpu6050_init()) != ESP_OK) {
        ESP_LOGE(HID_JOYSTICK_TAG, "%s init read mpu6050 failed\n", __func__);
    }*/
    if((ret = read_joystick_init()) != ESP_OK) {
        ESP_LOGE(HID_JOYSTICK_TAG, "%s init read joystick failed\n", __func__);
    }
}

