/**
 * Brief
 * This example shows how to connect an ESP device to the Bytebeam cloud.
 * This is good starting point if you are looking to remotely update your ESP device
   or push a command to it or stream data from it and visualise it on the Bytebeam Cloud.
*/

#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>

#include "esp_wifi.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "nvs.h"
#include "esp_log.h"
#include "esp_crc.h"
#include "esp_mac.h"
#include "esp_now.h"
#include "esp_crc.h"

#include "espnow_example.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"


#include "driver/gpio.h"
#include "driver/i2c.h"


#include "sht31.h"


// Global copy of gateway
esp_now_peer_info_t gateway;

#define PRINTSCANRESULTS 0
#define DELETEBEFOREPAIR 0
#define ESPNOW_CHANNEL   1

// this macro is used to specify the delay for 1 sec.
#define APP_DELAY_ONE_SEC 500u

static int config_publish_period = APP_DELAY_ONE_SEC;

static float temperature = 0.0;
static float humidity = 90;

static const char *TAG = "BYTEBEAM_TEMP_HUMID_EXAMPLE";


enum DeviceType {
    DT_Gateway        = 1,
    DT_Node        
};


struct SensorPayload {
    uint8_t sensor_profile;           // identifies sensor profile
    uint8_t hwRev;                  // identifies hw revision
    uint8_t fwRev;                 // identifies fw revision
    uint8_t deviceType;           // identifies device type            
    uint8_t batteryPercentage;  //batteryPercentage of nodes
    float temperature;
    float humidity;
}__attribute__ ((packed));

struct SensorPayload sensor_payload;


void InitESPNow() {
  // Disconnect from WiFi.
  esp_wifi_disconnect();

  // Initialize ESPNow.
  if (esp_now_init() == ESP_OK) {
    // ESPNow initialized successfully.
    printf("ESPNow Init Success\n");
  } else {
    // ESPNow initialization failed. Restart the device.
    printf("ESPNow Init Failed\n");
    ESP.restart();
  }
}

static void get_sht_values(void)
{
    int ret_val;

    ret_val = sht31_read_temp_humi(&temperature, &humidity);
    sensor_payload.temperature = temperature;
    sensor_payload.humidity = humidity;
    sensor_payload.batteryPercentage = 100;
    sensor_payload.sensor_profile = 1;
    sensor_payload.hwRev = 1;
    sensor_payload.fwRev = 1;
    sensor_payload.deviceType = 2;
    
    
    if(ret_val != 0)
    {
        ESP_LOGE(TAG, "Failed to read sht values.");
    }
    
    ESP_LOGI(TAG, "SHT3x Sensor : %.2f °C, %.2f %% \n",temperature, humidity);
}

static void sht_init(void)
{
    int ret_val;

    ret_val = sht31_init();

    if(ret_val != 0)
    {
        ESP_LOGE(TAG, "Failed to initialized sht.");
    }
}

static void ScanForgateway() {
  // Scan for WiFi networks in AP mode
  wifi_scan_config_t scan_config = {
    .ssid = "",
    .bssid = {0},
    .channel = ESPNOW_CHANNEL,
    .scan_type = WIFI_SCAN_TYPE_ACTIVE,
    .max_scan_results = 0,
    .scan_time = 300,
  };

  int16_t scan_results = esp_wifi_scan_start(&scan_config, true);

  // Check if the scan was successful
  if (scan_results == ESP_OK) {
    // Clear the gateway information
    memset(&gateway, 0, sizeof(gateway));

    // Iterate over the scan results
    for (int i = 0; i < scan_results; ++i) {
      // Get the SSID and RSSI of the current device
      char ssid[32];
      uint8_t bssid[6];
      int32_t rssi;
      esp_wifi_scan_get_results_with_ts(i, ssid, sizeof(ssid), bssid, &rssi, NULL);

      // Check if the current device starts with `gateway`
      if (strcmp(ssid, "Gateway-") == 0) {
        // SSID of interest
        ESP_LOGI("ESPNow", "Found a gateway");

        // Get the MAC address of the gateway
        for (int ii = 0; ii < 6; ++ii) {
          gateway.peer_addr[ii] = bssid[ii];
        }

        gateway.channel = ESPNOW_CHANNEL; // pick a channel
        gateway.encrypt = 0; // no encryption

        // We are planning to have only one gateway in this example, so break after we find one.
        break;
      }
    }
  } else {
    ESP_LOGE("ESPNow", "Scan failed");
  }

  // Stop the scan
  esp_wifi_scan_stop();
}

bool manageGateway() {
  if (gateway.channel == ESPNOW_CHANNEL) {
    if (DELETEBEFOREPAIR) {
      deletePeer();
    }

    ESP_LOGI("ESPNow", "Gateway Status: ");

    // Check if the peer exists
    bool exists = esp_now_is_peer_exist(gateway.peer_addr);
    if (exists) {
      // Gateway already paired.
      ESP_LOGI("ESPNow", "Already Paired");
      return true;
    } else {
      // Gateway not paired, attempt pair
      esp_err_t addStatus = esp_now_add_peer(&gateway);
      if (addStatus == ESP_OK) {
        // Pair success
        ESP_LOGI("ESPNow", "Pair success");
        return true;
      } else if (addStatus == ESP_ERR_ESPNOW_NOT_INIT) {
        // How did we get so far!!
        ESP_LOGE("ESPNow", "ESPNOW Not Init");
        return false;
      } else if (addStatus == ESP_ERR_ESPNOW_ARG) {
        ESP_LOGE("ESPNow", "Invalid Argument");
        return false;
      } else if (addStatus == ESP_ERR_ESPNOW_FULL) {
        ESP_LOGE("ESPNow", "Peer list full");
        return false;
      } else if (addStatus == ESP_ERR_ESPNOW_NO_MEM) {
        ESP_LOGE("ESPNow", "Out of memory");
        return false;
      } else if (addStatus == ESP_ERR_ESPNOW_EXIST) {
        ESP_LOGE("ESPNow", "Peer Exists");
        return true;
      } else {
        ESP_LOGE("ESPNow", "Not sure what happened");
        return false;
      }
    }
  } else {
    // No gateway found to process
    ESP_LOGE("ESPNow", "No gateway found to process");
    return false;
  }
}


void deletePeer() {
  esp_err_t delStatus = esp_now_del_peer(gateway.peer_addr);
  ESP_LOGI("ESPNow", "Gateway Delete Status: ");
  if (delStatus == ESP_OK) {
    // Delete success
    ESP_LOGI("ESPNow", "Success");
  } else if (delStatus == ESP_ERR_ESPNOW_NOT_INIT) {
    // How did we get so far!!
    ESP_LOGE("ESPNow", "ESPNOW Not Init");
  } else if (delStatus == ESP_ERR_ESPNOW_ARG) {
    ESP_LOGE("ESPNow", "Invalid Argument");
  } else if (delStatus == ESP_ERR_ESPNOW_NOT_FOUND) {
    ESP_LOGE("ESPNow", "Peer not found.");
  } else {
    ESP_LOGE("ESPNow", "Not sure what happened");
  }
}

void sendData() {
  get_sht_values();
  ESP_LOGI("ESPNow", "Temp= %.1f, battery= %d, humidity=%.1f, HW_REV=%d, sensor_profile=%d\n",sensor_payload.temperature, sensor_payload.batteryPercentage, sensor_payload.humidity,1,sensor_payload.sensor_profile);
  const uint8_t *peer_addr = gateway.peer_addr;
  esp_err_t result = esp_now_send(peer_addr, &sensor_payload, sizeof(sensor_payload));
  ESP_LOGI("ESPNow", "Send Status: ");
  if (result == ESP_OK) {
    ESP_LOGI("ESPNow", "Success");
  } else if (result == ESP_ERR_ESPNOW_NOT_INIT) {
    // How did we get so far!!
    ESP_LOGE("ESPNow", "ESPNOW not Init.");
  } else if (result == ESP_ERR_ESPNOW_ARG) {
    ESP_LOGE("ESPNow", "Invalid Argument");
  } else if (result == ESP_ERR_ESPNOW_INTERNAL) {
    ESP_LOGE("ESPNow", "Internal Error");
  } else if (result == ESP_ERR_ESPNOW_NO_MEM) {
    ESP_LOGE("ESPNow", "ESP_ERR_ESPNOW_NO_MEM");
  } else if (result == ESP_ERR_ESPNOW_NOT_FOUND) {
    ESP_LOGE("ESPNow", "Peer not found.");
  } else {
    ESP_LOGE("ESPNow", "Not sure what happened");
  }
}

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  // Convert the MAC address to a string.
  char mac_str[18];
  snprintf(mac_str, sizeof(mac_str), "%02x:%02x:%02x:%02x:%02x:%02x", mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);

  // Print the MAC address and the send status.
  printf("Last Packet Sent to: %s\n", mac_str);
  printf("Last Packet Send Status: %s\n", status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void goToSleep() {
  // Disconnect from WiFi.
  esp_wifi_disconnect();

  // Set WiFi mode to off.
  esp_wifi_set_mode(WIFI_MODE_OFF);

  // Print a message to indicate that the device is going to sleep.
  printf("Going to sleep\n");

  // Enable timer wakeup with a timeout of DEEPSLEEP_SECS seconds.
  esp_sleep_enable_timer_wakeup(DEEPSLEEP_SECS * MICRO_SECS_MULITPLIER);

  // Power down the RTC peripherals.
  esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_OFF);

  // Enter deep sleep mode.
  esp_deep_sleep_start();
}

static void app_loop()
{
    while (1)
    {   
        ScanForgateway();
        if (gateway.channel == ESPNOW_CHANNEL) {
            // `gateway` is defined
            // Add gateway as peer if it has not been added already
            bool isPaired = managegateway();
            if (isPaired) {
                // pair success or already paired
                // Send data to device
                sendData();
            } else {
                  // gateway pair failed
                  printf("gateway pair failed!");
      }
    }   
    vTaskDelay(config_publish_period / portTICK_PERIOD_MS);
    break; 
  }
  goToSleep();
}

void app_main(void)
{
    
  ESP_ERROR_CHECK(nvs_flash_init());
  ESP_ERROR_CHECK(esp_netif_init());
  ESP_ERROR_CHECK(esp_event_loop_create_default());
  // initialize the sht module
  sht_init();
  // Set the WiFi mode to station mode.
  esp_wifi_set_mode(WIFI_MODE_STA);

  // Set the WiFi channel to ESPNOW_CHANNEL.
  esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);

  // Initialize ESPNow.
  InitESPNow();

  // Register for the send callback.
  esp_now_register_send_cb(OnDataSent);

  // Print the MAC address of the master in station mode.
  printf("STA MAC: %s\n", esp_wifi_get_mac(ESP_IF_WIFI_STA));

  // Print the WiFi channel of the master in station mode.
  printf("STA CHANNEL: %d\n", esp_wifi_get_channel(ESP_IF_WIFI_STA));

  app_loop();

}
