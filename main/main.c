#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <stdlib.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_err.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_wifi.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"
#include "esp_timer.h"
#include "driver/ledc.h"
#include "server.h"
// ---------- Config ----------
static const char *TAG = "DEVICE_SCAN";

#define DEFAULT_SCAN_LIST_SIZE 20
#define MAX_WIFI_DEVICES 10
#define MAX_BT_DEVICES 10
#define RSSI_HISTORY_SIZE 10  // Store last 5 RSSI values for smoothing
#define HAMPEL_THRESHOLD 3.0f // Outlier threshold (standard deviations)
#define MAD_SCALE 1.4826f     // Scaling factor for MAD to approximate standard deviation
#define SAMPLES_PER_ANGLE 5
#define SERVO1_GPIO 19 // Servo 1: Clockwise
#define SERVO2_GPIO 18 // Servo 2: Anticlockwise

#define WIFI_SSID "Your_WIFI_Name"  // replace with your phone hotspot SSID
#define WIFI_PASS "Your Wifi Password" // replace with your hotspot password

static const char *TAG1 = "WIFI";
// ---------- Globals ----------
wifi_device_t wifi_devices[MAX_WIFI_DEVICES];
bt_device_t bt_devices[MAX_BT_DEVICES];
int g_wifi_count = 0;
int g_bt_count = 0;
float wifi_baseline_noise = 0.0f;
float bt_baseline_noise = 0.0f;
static bool is_scanning = false; // Track BLE scan state
target_device target;
bool start = false;
/* Event handler for Wi-Fi events */
static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START)
    {
        esp_wifi_connect();
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED)
    {
        ESP_LOGI(TAG1, "Disconnected. Reconnecting...");
        esp_wifi_connect();
        start = false;
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
    {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        start = true;
        ESP_LOGI(TAG1, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
    }
}

/* Initialize and connect to Wi-Fi in STA mode */
void wifi_init_sta(void)
{
    // Initialize TCP/IP stack and default event loop
    esp_netif_init();
    esp_event_loop_create_default();

    //  Create default Wi-Fi station
    esp_netif_create_default_wifi_sta();

    //  Initialize Wi-Fi with default config
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);

    //  Register event handlers
    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, &instance_any_id);
    esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, &instance_got_ip);

    //  Configure Wi-Fi credentials
    wifi_config_t wifi_config = {0};
    strcpy((char *)wifi_config.sta.ssid, WIFI_SSID);
    strcpy((char *)wifi_config.sta.password, WIFI_PASS);

    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_set_config(WIFI_IF_STA, &wifi_config);

    //  Start Wi-Fi
    esp_wifi_start();

    ESP_LOGI(TAG1, "Connecting to Wi-Fi SSID: %s", WIFI_SSID);
}
// ---------- Helper Functions ----------
bool mac_equal(const uint8_t *a, const uint8_t *b)
{
    if (a == NULL || b == NULL)
        return false;
    return memcmp(a, b, 6) == 0;
}

double calculate_distance(wifi_device_t wifi_device)
{
    return pow(10.0, ((-59.0 - (double)wifi_device.smoothed_rssi) / (10.0 * 3.0))); // Explicit cast
}
double calculate_distance2(bt_device_t bt_device)
{
    return pow(10.0, ((-59.0 - (double)bt_device.smoothed_rssi) / (10.0 * 3.0))); // Explicit cast
}
static void print_founded_device(void)
{
    bool wifi_available = (g_wifi_count > 0);
    bool ble_available = (g_bt_count > 0);

    if (!wifi_available && !ble_available)
    {
        ESP_LOGI(TAG, "Context: No devices found");
        return;
    }

    // Track which Wi-Fi and BLE devices have been fused
    bool wifi_fused[MAX_WIFI_DEVICES] = {false};
    bool ble_fused[MAX_BT_DEVICES] = {false};

    // First: fuse devices that exist in both Wi-Fi and BLE
    for (int i = 0; i < g_wifi_count; i++)
    {
        for (int j = 0; j < g_bt_count; j++)
        {
            // Compare by MAC or SSID/Name
            if (mac_equal(wifi_devices[i].mac, bt_devices[j].mac))
            {
                int fused_rssi = (int)(0.6f * wifi_devices[i].smoothed_rssi +
                                       0.4f * bt_devices[j].smoothed_rssi);

                ESP_LOGI(TAG, "founded device: SSID=%s / Name=%s Wi-Fi=%d BLE=%d → Fused RSSI=%d",
                         wifi_devices[i].ssid,
                         bt_devices[j].name,
                         wifi_devices[i].smoothed_rssi,
                         bt_devices[j].smoothed_rssi,
                         fused_rssi);

                wifi_fused[i] = true;
                ble_fused[j] = true;
                break; // Stop after first match
            }
        }
    }

    // Print Wi-Fi devices not fused
    for (int i = 0; i < g_wifi_count; i++)
    {
        if (!wifi_fused[i])
        {
            ESP_LOGI(TAG, "Wi-Fi only: SSID=%s RSSI=%d", wifi_devices[i].ssid, wifi_devices[i].smoothed_rssi);
        }
    }

    // Print BLE devices not fused
    for (int j = 0; j < g_bt_count; j++)
    {
        if (!ble_fused[j])
        {
            ESP_LOGI(TAG, "BLE only: Name=%s RSSI=%d", bt_devices[j].name, bt_devices[j].smoothed_rssi);
        }
    }
}

static float compute_median(int32_t *data, int count)
{
    if (count == 0)
        return 0.0f;

    int32_t temp[RSSI_HISTORY_SIZE];
    for (int i = 0; i < count; i++)
    {
        temp[i] = data[i];
    }

    // Insertion sort
    for (int i = 1; i < count; i++)
    {
        int32_t key = temp[i];
        int j = i - 1;
        while (j >= 0 && temp[j] > key)
        {
            temp[j + 1] = temp[j];
            j--;
        }
        temp[j + 1] = key;
    }

    if (count % 2 == 0)
    {
        return (float)(temp[count / 2 - 1] + temp[count / 2]) / 2.0f;
    }
    else
    {
        return (float)temp[count / 2];
    }
}

static float compute_median_float(float *data, int count)
{
    if (count == 0)
        return 0.0f;

    float temp[RSSI_HISTORY_SIZE];
    for (int i = 0; i < count; i++)
    {
        temp[i] = data[i];
    }

    // Insertion sort
    for (int i = 1; i < count; i++)
    {
        float key = temp[i];
        int j = i - 1;
        while (j >= 0 && temp[j] > key)
        {
            temp[j + 1] = temp[j];
            j--;
        }
        temp[j + 1] = key;
    }

    if (count % 2 == 0)
    {
        return (temp[count / 2 - 1] + temp[count / 2]) / 2.0f;
    }
    else
    {
        return temp[count / 2];
    }
}
// Initialize Kalman filter
static void kalman_init(kalman_state_t *kf, float q, float r, float initial_value)
{
    kf->q = q;
    kf->r = r;
    kf->x = initial_value;
    kf->p = 1.0f;
    kf->k = 0.0f;
    kf->initialized = true;
}
// Apply Kalman filter to measurement
static float kalman_update(kalman_state_t *kf, float measurement)
{
    if (!kf->initialized)
    {
        kf->x = measurement;
        kf->p = 1.0f;
        kf->initialized = true;
    }

    // Prediction update
    kf->p = kf->p + kf->q;

    // Measurement update
    kf->k = kf->p / (kf->p + kf->r);
    kf->x = kf->x + kf->k * (measurement - kf->x);
    kf->p = (1 - kf->k) * kf->p;

    return kf->x;
}

// ---------- Smooth RSSI Function ----------
static int32_t smooth_rssi(int32_t *rssi_history, int rssi_count, kalman_state_t *kf)
{
    if (rssi_count == 0)
        return 0;

    int valid_count = rssi_count > RSSI_HISTORY_SIZE ? RSSI_HISTORY_SIZE : rssi_count;

    // Median Filter
    float median = compute_median(rssi_history, valid_count);

    //  Hampel Filter
    float mad_data[RSSI_HISTORY_SIZE];
    for (int i = 0; i < valid_count; i++)
    {
        mad_data[i] = fabsf((float)rssi_history[i] - median);
    }
    float mad = compute_median_float(mad_data, valid_count) * MAD_SCALE;
    if (mad == 0)
        return (int32_t)median;
    int32_t cleaned[RSSI_HISTORY_SIZE];
    int cleaned_count = 0;

    for (int i = 0; i < valid_count; i++)
    {
        if (mad > 0 && fabsf((float)rssi_history[i] - median) > HAMPEL_THRESHOLD * mad)
        {
            cleaned[cleaned_count++] = (int32_t)median;
        }
        else
        {
            cleaned[cleaned_count++] = rssi_history[i];
        }
    }

    // Aggregate filtered values (median of cleaned)
    int32_t filtered_rssi = (int32_t)compute_median(cleaned, cleaned_count);

    //  Kalman Filter
    float smoothed = kalman_update(kf, (float)filtered_rssi);

    return (int32_t)smoothed;
}

static void update_wifi_rssi(wifi_ap_record_t *recs, uint16_t ap_num)
{
    // Step 1: Update RSSI history for existing or new devices

    for (int i = 0; i < ap_num; ++i)
    {
        bool exists = false;
        for (int j = 0; j < g_wifi_count; ++j)
        {
            if (target.is_direction && j != target.wifi_index)
            {
                continue;
            }
            if (mac_equal(recs[i].bssid, wifi_devices[j].mac))
            {
                exists = true;
                // Shift RSSI history and add new value
                if (wifi_devices[j].rssi_count < RSSI_HISTORY_SIZE)
                {
                    wifi_devices[j].rssi_history[wifi_devices[j].rssi_count++] = recs[i].rssi;
                }
                else
                {
                    // Shift left and add new RSSI
                    for (int k = 1; k < RSSI_HISTORY_SIZE; k++)
                    {
                        wifi_devices[j].rssi_history[k - 1] = wifi_devices[j].rssi_history[k];
                    }
                    wifi_devices[j].rssi_history[RSSI_HISTORY_SIZE - 1] = recs[i].rssi;
                }
                // Update SSID and channel
                strncpy(wifi_devices[j].ssid, (const char *)recs[i].ssid, sizeof(wifi_devices[j].ssid) - 1);
                wifi_devices[j].ssid[sizeof(wifi_devices[j].ssid) - 1] = '\0';
                wifi_devices[j].channel = recs[i].primary;
                wifi_devices[j].smoothed_rssi = smooth_rssi(wifi_devices[j].rssi_history, wifi_devices[j].rssi_count, &wifi_devices[j].kf);

                break;
            }
        }
        if (!exists && g_wifi_count < MAX_WIFI_DEVICES)
        {
            wifi_device_t d = {0};
            strncpy(d.ssid, (const char *)recs[i].ssid, sizeof(d.ssid) - 1);
            d.ssid[sizeof(d.ssid) - 1] = '\0';
            memcpy(d.mac, recs[i].bssid, 6);
            d.channel = recs[i].primary;
            d.rssi_history[0] = recs[i].rssi;
            d.rssi_count = 1;
            d.smoothed_rssi = recs[i].rssi; // Initial value
            kalman_init(&d.kf, 0.01f, 1.0f, d.rssi_history[0]);
            // d.angle_history = NULL;
            if (mac_equal(d.mac, target.target_bssi))
            {
                target.wifi_index = g_wifi_count;
            }
            wifi_devices[g_wifi_count++] = d;
        }
    }
}
static void update_bt_rssi(esp_ble_gap_cb_param_t *p)
{
    int exist = 0;
    // Dedup by MAC and update RSSI history for existing or new devices
    for (int i = 0; i < g_bt_count; ++i)
    {
        if (target.is_direction && i != target.b_index)
        {
            continue;
        }
        if (mac_equal(p->scan_rst.bda, bt_devices[i].mac))
        {
            exist = 1;
            // Shift RSSI history and add new value
            if (bt_devices[i].rssi_count < RSSI_HISTORY_SIZE)
            {
                bt_devices[i].rssi_history[bt_devices[i].rssi_count++] = p->scan_rst.rssi;
            }
            else
            {
                // Shift left and add new RSSI
                for (int k = 1; k < RSSI_HISTORY_SIZE; k++)
                {
                    bt_devices[i].rssi_history[k - 1] = bt_devices[i].rssi_history[k];
                }
                bt_devices[i].rssi_history[RSSI_HISTORY_SIZE - 1] = p->scan_rst.rssi;
            }
            bt_devices[i].smoothed_rssi = smooth_rssi(bt_devices[i].rssi_history, bt_devices[i].rssi_count, &bt_devices[i].kf);
            return;
        }
    }
    if (!exist && g_bt_count < MAX_BT_DEVICES)
    {
        // New device
        bt_device_t d = {0};
        memcpy(d.mac, p->scan_rst.bda, 6);
        d.rssi_history[0] = p->scan_rst.rssi;
        d.rssi_count = 1;
        d.smoothed_rssi = p->scan_rst.rssi; // Initial value
        kalman_init(&d.kf, 0.01f, 1.0f, d.rssi_history[0]);
        // Try complete then short name
        uint8_t len = 0;
        uint8_t *nm = esp_ble_resolve_adv_data(p->scan_rst.ble_adv, ESP_BLE_AD_TYPE_NAME_CMPL, &len);
        if (!nm || len == 0)
            nm = esp_ble_resolve_adv_data(p->scan_rst.ble_adv, ESP_BLE_AD_TYPE_NAME_SHORT, &len);
        if (nm && len)
        {
            if (len >= sizeof d.name)
                len = sizeof d.name - 1;
            memcpy(d.name, nm, len);
            d.name[len] = '\0';
        }
        else
        {
            strncpy(d.name, "(no name)", sizeof d.name - 1);
        }
        bt_devices[g_bt_count++] = d;
        if (mac_equal(bt_devices[g_bt_count - 1].mac, target.target_ble))
        {
            target.b_index = g_bt_count - 1;
        }
    }
}
static void wifi_scan_once();

void estimate_direction_for_target(target_device target, int mode, int angle)
{
    // mode: 1 = Wi-Fi, 2 = BLE
    //angle = (angle / 10);
    float rssi_samples[SAMPLES_PER_ANGLE];
    int sample_count = 0;
    for (int s = 0; s < SAMPLES_PER_ANGLE; s++)
    {
        ESP_LOGI(TAG," went in estimte direction b4 if");
        if (mode == 1)
        {
            // Wi-Fi: scan and update RSSI for devices
            ESP_LOGI(TAG," went in estimte direction b4 wifiscanonce");
            wifi_scan_once();
            ESP_LOGI(TAG," went in estimte direction after wifiscanonce");
            // Find target SSID in scanned list
            rssi_samples[sample_count++] = wifi_devices[target.wifi_index].smoothed_rssi;
        }
        else if (mode == 2)
        {
            rssi_samples[sample_count++] = bt_devices[target.b_index].smoothed_rssi;
        }
        else
        {
            int fused_rssi = (int)(0.6f * wifi_devices[target.wifi_index].smoothed_rssi + 0.4f * bt_devices[target.b_index].smoothed_rssi);
            rssi_samples[sample_count++] = fused_rssi;
        }

        vTaskDelay(pdMS_TO_TICKS(20)); // short delay between samples

        if (sample_count == 0)
        {
            target.angle_history[angle].avg_rssi = -100; // mark as undetected
            target.angle_history[angle].variance = 0;
            target.angle_history[angle].los = false;
            target.angle_history[angle].confidence = 0;
            continue;
        }
    }
    // Compute average and variance for this angle
    float sum = 0, var = 0;
    for (int i = 0; i < sample_count; i++)
        sum += rssi_samples[i];
    float avg = sum / sample_count;
    for (int i = 0; i < sample_count; i++)
        var += (rssi_samples[i] - avg) * (rssi_samples[i] - avg);
    var /= sample_count;

    // LOS/NLOS detection: low variance = likely LOS
    bool los_flag = (var < 5.0f);                      // threshold, tune experimentally
    float confidence = avg + (los_flag ? 5.0f : 0.0f); // boost confidence for LOS

    // Store results
    target.angle_history[angle].angle = angle;
    target.angle_history[angle].avg_rssi = avg;
    target.angle_history[angle].variance = var;
    target.angle_history[angle].los = los_flag;
    target.angle_history[angle].confidence = confidence;
    target.angle_history[angle].valid = true;
    
}

// ---------- NVS ----------
static void initialize_nvs(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
}

// ---------- Wi-Fi ----------
static void initialize_wifi(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
}

static void wifi_scan_once()
{

    for (int ch_idx = 0; ch_idx < target.num_channels; ch_idx++)
    {
        uint8_t ch = target.channels[ch_idx];

        if (ch == 0)
        {
            continue;
        }
        // ch = target.target_channel ? target.target_channel : ch;
        wifi_scan_config_t scan_cfg = {
            .ssid = NULL,  // target.target_ssid ? target.target_ssid : NULL,
            .bssid = NULL, // target.target_bssi ? target.target_bssi : NULL,
            .channel = ch,
            .show_hidden = true,
            .scan_type = WIFI_SCAN_TYPE_ACTIVE,
            .scan_time.active = {.min = 200, .max = 200} // us slots per channel
        };

        ESP_LOGI(TAG, "Scanning on channel %d", ch);
        ESP_ERROR_CHECK(esp_wifi_scan_start(&scan_cfg, true));
        uint16_t ap_num = 0;
        ESP_ERROR_CHECK(esp_wifi_scan_get_ap_num(&ap_num));
        if (ap_num == 0)
        {
            ESP_LOGI(TAG, "Wi-Fi: no APs on channel %d", ch);
            target.channels[ch_idx] = 0;
            continue;
        }

        wifi_ap_record_t *recs = calloc(ap_num, sizeof(*recs));
        if (!recs)
        {
            ESP_LOGE(TAG, "Wi-Fi: calloc %u failed", (unsigned)ap_num);
            continue;
        }

        ESP_ERROR_CHECK(esp_wifi_scan_get_ap_records(&ap_num, recs));
        update_wifi_rssi(recs, ap_num);
        if (target.is_direction && ch == target.target_channel)
        {
            break;
        }
    }
}
// ===================== BLE
static void ble_gap_cb(esp_gap_ble_cb_event_t e, esp_ble_gap_cb_param_t *p)
{
    switch (e)
    {
    case ESP_GAP_BLE_SCAN_START_COMPLETE_EVT:
        is_scanning = true;
        ESP_LOGI(TAG, "BLE scan start: %s", is_scanning ? "OK" : "FAIL");
        break;
    case ESP_GAP_BLE_SCAN_RESULT_EVT:
        if (p->scan_rst.search_evt == ESP_GAP_SEARCH_INQ_RES_EVT)
        {

            update_bt_rssi(p);
        }
        break;
    case ESP_GAP_BLE_SCAN_STOP_COMPLETE_EVT:
        is_scanning = false;
        ESP_LOGI(TAG, "BLE scan stop: %s", (p->scan_stop_cmpl.status == ESP_BT_STATUS_SUCCESS) ? "OK" : "ERR");
        break;
    default:
        break;
    }
}

static void ble_init(void)
{
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));
    ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_BLE));
    ESP_ERROR_CHECK(esp_bluedroid_init());
    ESP_ERROR_CHECK(esp_bluedroid_enable());
    ESP_ERROR_CHECK(esp_ble_gap_register_callback(ble_gap_cb));

    esp_ble_scan_params_t p = {
        .scan_type = BLE_SCAN_TYPE_ACTIVE,
        .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
        .scan_filter_policy = BLE_SCAN_FILTER_ALLOW_ALL,
        .scan_interval = 0x100,
        .scan_window = 0x50,
        .scan_duplicate = BLE_SCAN_DUPLICATE_DISABLE,
    };
    ESP_ERROR_CHECK(esp_ble_gap_set_scan_params(&p));
}

void start_ble_scan_continous(uint32_t duration_sec)
{
    g_bt_count = 0; // reset count
    ESP_ERROR_CHECK(esp_ble_gap_start_scanning(duration_sec));
}

// servo section
uint32_t angle_to_duty(int angle)
{
    uint32_t pulse_width_us = 500 + (angle * 2000 / 180); // Changed to uint32_t
    return (pulse_width_us * 65535) / 20000;              // Convert to 16-bit duty for 20ms period (50Hz)
}

void init_servos()
{
    ledc_timer_config_t timer = {
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .timer_num = LEDC_TIMER_0,
        .duty_resolution = LEDC_TIMER_16_BIT,
        .freq_hz = 50,
        .clk_cfg = LEDC_AUTO_CLK};
    ledc_timer_config(&timer);

    ledc_channel_config_t servo1 = {
        .gpio_num = SERVO1_GPIO,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
        .hpoint = 0};
    ledc_channel_config(&servo1);

    ledc_channel_config_t servo2 = {
        .gpio_num = SERVO2_GPIO,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .channel = LEDC_CHANNEL_1,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
        .hpoint = 0};
    ledc_channel_config(&servo2);
}

void sweep_servo(int start_angle, int end_angle, int channel, int delay, int mode)
{
    int step = (start_angle < end_angle) ? 10 : -10;

    for (int angle = start_angle; angle != end_angle + step; angle += step)
    {
        uint32_t duty = angle_to_duty(angle);
        ledc_set_duty(LEDC_HIGH_SPEED_MODE, channel, duty);
        ledc_update_duty(LEDC_HIGH_SPEED_MODE, channel);
        vTaskDelay(pdMS_TO_TICKS(delay));
        if (target.is_direction) // note set teh direstion after a cycle giving the target
        {
            ESP_LOGI(TAG,"Went in sweep servo");
            estimate_direction_for_target(target, mode, angle);
        }
        else
        {
            if (mode == 1) // Wi-Fi
            {
                wifi_scan_once();
            }
            else if (mode == 2) // BLE
            {
                ESP_LOGI(TAG, "Servo at %d° - BLE devices detected: %d", angle, g_bt_count);
                vTaskDelay(pdMS_TO_TICKS(delay));
            }
        }
    }
}

void sweep_servos(int start, int end, int delay, int rot, int mode)
{

    if (start > 180 && end > 180)
    {
        sweep_servo(0, 180, LEDC_CHANNEL_0, 0, mode);
        int temp = start - 180;
        sweep_servo(0, temp, LEDC_CHANNEL_1, 0, mode);
        int y = end - start;
        for (int i = 0; i < rot; i++)
        {
            sweep_servo(temp, y, LEDC_CHANNEL_1, delay, mode);
            int temp1 = temp;
            temp = y;
            y = temp1;
        }
    }
    else if (start < 180 && end < 180)
    {
        for (int i = 0; i < rot; i++)
        {
            sweep_servo(start, end, LEDC_CHANNEL_0, delay, mode);
            int temp1 = start;
            start = end;
            end = temp1;
        }
    }
    else if (start < 180 && end > 180)
    {
        int y = end - 180;
        int k = 180, l = 0;
        for (int i = 0; i < rot; i++)
        {
            sweep_servo(start, k, LEDC_CHANNEL_0, delay, mode);
            sweep_servo(l, y, LEDC_CHANNEL_1, delay, mode);
            int temp1 = start;
            start = k;
            k = temp1;
            temp1 = y;
            y = l;
            l = temp1;
        }
    }
    if (mode == 2 && is_scanning)
    {
        ESP_ERROR_CHECK(esp_ble_gap_stop_scanning());
        is_scanning = false;
    }
}
void initialize_target()
{
    static uint8_t channels[] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13};
    target.channels = channels;
    target.num_channels = sizeof(channels);
    target.is_direction = false;
    target.target_channel = -1;
    target.target_ssid = NULL;
    target.target_bssi = NULL;
    target.target_ble = NULL;
    target.angle_history = calloc(37, sizeof(angle_info_t));
    target.wifi_index = -1;
    target.b_index = -1;
}

void set_target_device(const unsigned char *wifi_ssid, uint8_t channel, const uint8_t *target_bssi, const uint8_t *target_ble, bool direction)
{
    target.is_direction = direction;
    target.target_channel = channel;

    // free previous memory (optional)
    if (target.target_ssid)
        free(target.target_ssid);
    if (target.target_bssi)
        free(target.target_bssi);
    if (target.target_ble)
        free(target.target_ble);

    target.target_ssid = malloc(33);
    target.target_bssi = malloc(6);
    target.target_ble = malloc(6);

    if (wifi_ssid)
        strncpy((char *)target.target_ssid, (const char *)wifi_ssid, 32);
    else
        target.target_ssid[0] = '\0';
    ((char *)target.target_ssid)[32] = '\0'; // ensure null-termination

    if (target_bssi)
        memcpy(target.target_bssi, target_bssi, 6);
    else
        memset(target.target_bssi, 0, 6);

    if (target_ble)
        memcpy(target.target_ble, target_ble, 6);
    else
        memset(target.target_ble, 0, 6);
}

int get_best_angle_for_target(void)
{
    if (target.angle_history == NULL)
        return -1; // Not initialized

    int best_angle = -1;
    float max_confidence = -9999;

    for (int i = 0; i < 37; i++)
    {
        if (!target.angle_history[i].valid)
            continue;  // skip empty or unfilled entries
        ESP_LOGI(TAG,"The angles are %d",target.angle_history[i].angle);
        float conf = target.angle_history[i].confidence;

        if (conf > max_confidence)
        {
            max_confidence = conf;
            best_angle = target.angle_history[i].angle;
        }
    }

    return best_angle;
}


void app_main(void)
{

    vTaskDelay(pdMS_TO_TICKS(4000)); // Delay 4s to stabilize serial output
    initialize_nvs();
    // initialize_wifi();

    ble_init();
    wifi_init_sta();
    init_servos();
    initialize_target();

    webserver_start();
    vTaskDelay(pdMS_TO_TICKS(1000));
    while (!start)
        ;
    start = false;
    vTaskDelay(pdMS_TO_TICKS(5000)); // Wait 5 seconds
    sweep_servos(0, 100, 10, 1, 1);  // Sweep servos with Wi-Fi scanning
                                     // vTaskDelay(pdMS_TO_TICKS(3000)); // Short delay before BLE scan
    start_ble_scan_continous(50);    // Start BLE scan for 20 seconds
    sweep_servos(0, 360, 100, 1, 2); // Sweep servos with BLE scanning
    // vTaskDelay(pdMS_TO_TICKS(2000)); // Short delay before cleanup

    // print_founded_device();
   
   //ESP_LOGI(TAG,"the angle for varunesh%d",);
}
