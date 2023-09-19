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
#include "esp_ota_ops.h"
#include "esp_http_client.h"
#include "esp_https_ota.h"
#include "esp_tls.h"
#include "esp_sntp.h"
#include "esp_log.h"
#include "mqtt_client.h"

#include "protocol_examples_common.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"

#include "lwip/sockets.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"

#include "cJSON.h"

#include "driver/gpio.h"
#include "driver/i2c.h"

#include "bytebeam_sdk.h"
#include "list_sensor_data.h"


// this macro is used to specify the delay for 1 sec.
#define APP_DELAY_ONE_SEC 1000u

static int config_publish_period = APP_DELAY_ONE_SEC;

static float temperature = 0.0;
static float humidity = 90;

static char sht_stream[] = "sht_stream";

static bytebeam_client_t bytebeam_client;

static const char *TAG = "BYTEBEAM_TEMP_HUMID_EXAMPLE";

struct Node *head = NULL;

static int publish_sht_values(bytebeam_client_t *bytebeam_client)
{   
    while (countNodes(head)>0){
      struct SensorPayload *payload = nullptr;
      struct Node* currentNode = head;
      payload = currentNode->payload;
      struct timeval te;
      long long milliseconds = 0;
      static uint64_t sequence = 0;
      cJSON *device_shadow_json_list = NULL;
      cJSON *device_shadow_json = NULL;
      cJSON *timestamp_json = NULL;
      cJSON *temperature_json = NULL;
      cJSON *humidity_json = NULL;
      char *node_id_json = NULL;
      char *string_json = NULL;
      device_shadow_json_list = cJSON_CreateArray();

      if (device_shadow_json_list == NULL)
      {
        ESP_LOGE(TAG, "Json Init failed.");
        return -1;
      }
      device_shadow_json = cJSON_CreateObject();

      if (device_shadow_json == NULL)
      {
          ESP_LOGE(TAG, "Json add failed.");
          cJSON_Delete(device_shadow_json_list);
          return -1;
      }
      // get current time
      gettimeofday(&te, NULL);
      milliseconds = te.tv_sec * 1000LL + te.tv_usec / 1000;
      timestamp_json = cJSON_CreateNumber(milliseconds);

      if (timestamp_json == NULL)
      {
          ESP_LOGE(TAG, "Json add time stamp failed.");
          cJSON_Delete(device_shadow_json_list);
          return -1;
      }
      cJSON_AddItemToObject(device_shadow_json, "timestamp", timestamp_json);

      node_id_json =  cJSON_CreateString(currentNode->mac_str);

      if (node_id_json == NULL)
      {
          ESP_LOGE(TAG, "Json add temperature failed.");
          cJSON_Delete(device_shadow_json_list);
          return -1;
      }
      
      cJSON_AddItemToObject(device_shadow_json, "node_id", node_id_json);

      temperature_json = cJSON_CreateNumber(payload->temperature);

      {
          ESP_LOGE(TAG, "Json add temperature failed.");
          cJSON_Delete(device_shadow_json_list);
          return -1;
      }
      
      cJSON_AddItemToObject(device_shadow_json, "temperature", temperature_json);

      humidity_json = cJSON_CreateNumber(payload->humidity);

      if (humidity_json == NULL)
      {
          ESP_LOGE(TAG, "Json add humidity failed.");
          cJSON_Delete(device_shadow_json_list);
          return -1;
      }
      
      cJSON_AddItemToObject(device_shadow_json, "humidity", humidity_json);

      cJSON_AddItemToArray(device_shadow_json_list, device_shadow_json);

      string_json = cJSON_Print(device_shadow_json_list);

      if(string_json == NULL)
        {
            ESP_LOGE(TAG, "Json string print failed.");
            cJSON_Delete(device_shadow_json_list);
            return -1;
        }

      ESP_LOGI(TAG, "\nStatus to send:\n%s\n", string_json);

      // publish the json to sht stream
    int ret_val = bytebeam_publish_to_stream(bytebeam_client, sht_stream, string_json);

    cJSON_Delete(device_shadow_json_list);
    cJSON_free(string_json);
    
    }
}

bool processQueuedData(bytebeam_client_t *bytebeam_client) {

  bool connOutcome = false;
  if (countNodes(head) < 10) {
    ESP_LOGE("NO data to process Try Again");
    return false;
  }
  
  if (esp_now_unregister_recv_cb() != ESP_OK) {
    ESP_LOGE("cant unregister ESP nOW cb");
  }

  esp_err_t esp_now_err = esp_now_deinit();
  if (esp_now_err != ESP_OK) {
    ESP_LOGE("ESPNow deinit failed");
  }

  switchToWifiClient();
    
  syncTimeFromNtp();

  // publish sht values
  ret_val = publish_sht_values(bytebeam_client);

  if (ret_val != 0)
  {
      ESP_LOGE(TAG, "Failed to publish sht values.");
  }

  storeAndReset();

  return true;
}

static void app_start(bytebeam_client_t *bytebeam_client)
{
    int ret_val = 0;

    while (1)
    {   
        processQueuedData(bytebeam_client);

        vTaskDelay(config_publish_period / portTICK_PERIOD_MS);
    }
}

static void time_sync_notification_cb(struct timeval *tv)
{
    ESP_LOGI(TAG, "Notification of a time synchronization event");
}

static void initialize_sntp(void)
{
    ESP_LOGI(TAG, "Initializing SNTP");

    sntp_setoperatingmode(SNTP_OPMODE_POLL);
    sntp_setservername(0, "pool.ntp.org");
    sntp_set_time_sync_notification_cb(time_sync_notification_cb);

#ifdef CONFIG_SNTP_TIME_SYNC_METHOD_SMOOTH
    sntp_set_sync_mode(SNTP_SYNC_MODE_SMOOTH);
#endif

    sntp_init();
}

static void sync_time_from_ntp(void)
{
    time_t now = 0;
    struct tm timeinfo = {0};
    int retry = 0;
    const int retry_count = 10;

    initialize_sntp();

    // wait for time to be set
    while (sntp_get_sync_status() == SNTP_SYNC_STATUS_RESET && ++retry < retry_count)
    {
        ESP_LOGI(TAG, "Waiting for system time to be set... (%d/%d)", retry, retry_count);
        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }

    time(&now);
    localtime_r(&now, &timeinfo);
}

bool configESPNowDeviceAP() {
  // Set the WiFi mode to AP and STA mode.
  esp_wifi_set_mode(WIFI_MODE_AP_STA);

  // Print a message to indicate that the device is being configured as an AP.
  ESP_LOGI("Configuring Device AP");

  // Get the MAC address of the device.
  uint8_t mac[6];
  esp_wifi_get_mac(ESP_IF_WIFI_STA, mac);

  // Get the last 3 bytes of the MAC address.
  char mac_str[18];
  snprintf(mac_str, sizeof(mac_str), "%02x%02x%02x", mac[3], mac[4], mac[5]);

  // Create the SSID for the AP.
  char ssid[32];
  snprintf(ssid, sizeof(ssid), "%s-%s", AP_MODE_SSID, mac_str);

  // Print the SSID.
  ESP_LOGI("Configuring to: %s", ssid);

  // Configure the device as an AP.
  bool result = esp_wifi_softap(ssid, "", ESPNOW_CHANNEL, 0);
  if (!result) {
    ESP_LOGE("AP Config failed.");
    return false;
  }

  return true;
}


bool switchToWifiClient() {
  // Check if the device is already connected to WiFi.
  if (esp_wifi_is_connected()) {
    return true;
  }

  // Reconnect to WiFi.
  /*bool result = reconnectWiFi(ssid, pass, maxWaitSecs);
  if (!result) {
    return false;
  }*/
   ESP_ERROR_CHECK(example_connect());

    // sync time from the ntp
    sync_time_from_ntp();
  // Return true.
  return true;
}

bool switchToESPNowGateway() {
  // Configure the device as an ESP-NOW gateway.
  configESPNowDeviceAP();

  // Disconnect from WiFi.
  esp_wifi_disconnect();

  // Initialize ESPNow.
  esp_err_t err = esp_now_init();
  if (err != ESP_OK) {
    ESP_LOGE("ESPNow Init failed");
    return false;
  } else {
    ESP_LOGI("ESPNow Init Success");
  }
  return true;
}

void OnDataRecv(const uint8_t *mac_addr, const uint8_t *data, int data_len) {
  // Convert the MAC address to a string.
  char mac_str[18];
  snprintf(mac_str, sizeof(mac_str), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);

  // Check if the data length is less than the size of the `SensorPayload` structure.
  if (data_len < sizeof(SensorPayload)) {
    ESP_LOGE("Bad data from espnow");
    return;
  }

  // Convert the data to a `SensorPayloadTH` structure.
  SensorPayload *payload = (SensorPayload*)data;

  // Print the temperature and sensor profile.
  ESP_LOGI("Temperature: %.1f", payload->temperature);
  ESP_LOGI("Sensor profile: %d", payload->sensor_profile);

  const char* mac_str_str = mac_str; 
  // Enqueue the sensor payload.
  addNode(&head, payload, mac_str_str);
}


void app_main(void)
{
    ESP_LOGI(TAG, "[APP] Startup..");
    ESP_LOGI(TAG, "[APP] IDF version: %s", esp_get_idf_version());

    esp_log_level_set("*", ESP_LOG_INFO);
    esp_log_level_set("MQTT_CLIENT", ESP_LOG_VERBOSE);
    esp_log_level_set("TRANSPORT_BASE", ESP_LOG_VERBOSE);
    esp_log_level_set("TRANSPORT", ESP_LOG_VERBOSE);
    esp_log_level_set("OUTBOX", ESP_LOG_VERBOSE);

    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

   

    // initialize the sht module
    sht_init();
    
    // setting up the device info i.e to be seen in the device shadow
    bytebeam_client.device_info.status           = "Device is Up!";
    bytebeam_client.device_info.software_type    = "temp-humid-app";
    bytebeam_client.device_info.software_version = "1.0.0";
    bytebeam_client.device_info.hardware_type    = "ESP32 DevKit V1";
    bytebeam_client.device_info.hardware_version = "rev1";

    switchToESPNowGateway();
    esp_now_register_recv_cb(OnDataRecv);

    //
    // start the main application
    //
    app_start(&bytebeam_client);
}
