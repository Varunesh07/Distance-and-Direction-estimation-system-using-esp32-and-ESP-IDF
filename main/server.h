
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

/*
 * NOTE: These struct definitions MUST match the ones in your main.c.
 * They are repeated here so webserver.c can access the globals.
 */

#define RSSI_HISTORY_SIZE 10
#define MAX_WIFI_DEVICES 10
#define MAX_BT_DEVICES 10

typedef struct
{
    float q; // Process noise covariance
    float r; // Measurement noise covariance
    float x; // Estimated value
    float p; // Estimation error covariance
    float k; // Kalman gain
    bool initialized;
} kalman_state_t;

typedef struct
{
    int angle;
    float avg_rssi;
    float variance;
    bool los;         // Line-of-sight flag
    float confidence; // Combined metric
    bool valid;
} angle_info_t;

typedef struct
{
    char ssid[33]; // 32 + null
    uint8_t mac[6];
    uint8_t channel;
    int32_t rssi_history[RSSI_HISTORY_SIZE]; // Store recent RSSI values
    int rssi_count;                          // Number of RSSI values stored
    int32_t smoothed_rssi;                   // Smoothed RSSI after filtering
    kalman_state_t kf;

} wifi_device_t;

typedef struct
{
    uint8_t mac[6];
    char name[32];
    int32_t rssi_history[RSSI_HISTORY_SIZE]; // Store recent RSSI values
    int rssi_count;                          // Number of RSSI values stored
    int32_t smoothed_rssi;                   // Smoothed RSSI after filtering
    kalman_state_t kf;

} bt_device_t;

typedef struct
{
    uint8_t *channels;
    int num_channels;
    uint8_t target_channel;
    unsigned char *target_ssid;
    uint8_t *target_bssi;
    uint8_t *target_ble;
    uint8_t wifi_index;
    uint8_t b_index;
    bool is_direction;
    angle_info_t *angle_history;

} target_device;

/* Globals from main.c (declared there) that we will read */
extern wifi_device_t wifi_devices[MAX_WIFI_DEVICES];
extern bt_device_t bt_devices[MAX_BT_DEVICES];
extern int g_wifi_count;
extern int g_bt_count;
extern target_device target;

/* Functions in main.c that webserver will call */
void set_target_device(const unsigned char *wifi_ssid, uint8_t channel, const uint8_t *target_bssi, const uint8_t *target_ble, bool direction);
double calculate_distance(wifi_device_t wifi_device); // uses wifi_device.smoothed_rssi
double calculate_distance2(bt_device_t bt_device);
int get_best_angle_for_target(void);
/* Start webserver (call from main.app_main) */
void webserver_start();
