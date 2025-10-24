#include "server.h"
#include <esp_http_server.h>

#include <esp_log.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

static const char *TAG = "WEBSERVER";

/* Small helpers: mac -> string */
static void mac_to_str(const uint8_t *mac, char *out, size_t out_len)
{
    if (!mac || !out)
    {
        strncpy(out, "(nil)", out_len);
        return;
    }
    snprintf(out, out_len, "%02X:%02X:%02X:%02X:%02X:%02X",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

/* Embedded HTML/CSS/JS */
static const char index_html[] =
    "<!doctype html>\n"
    "<html>\n"
    "<head>\n"
    "  <meta charset='utf-8'>\n"
    "  <meta name='viewport' content='width=device-width,initial-scale=1'>\n"
    "  <title>ESP Device Scanner</title>\n"
    "  <style>\n"
    "    body { font-family: Arial, Helvetica, sans-serif; padding: 12px; }\n"
    "    .device { padding:8px; border:1px solid #ddd; margin:6px 0; display:flex; justify-content:space-between; align-items:center; }\n"
    "    .btn { padding:6px 10px; border-radius:4px; cursor:pointer; border: 1px solid #888; background:#f6f6f6; }\n"
    "    #distance { margin-top: 12px; font-weight:bold; }\n"
    "  </style>\n"
    "</head>\n"
    "<body>\n"
    "  <h2>Available devices</h2>\n"
    "  <div id='list'>Loading...</div>\n"
    "  <div id='distance'></div>\n"
    "  <script>\n"
    "async function fetchDevices(){\n"
    "  const res = await fetch('/devices');\n"
    "  const json = await res.json();\n"
    "  const list = document.getElementById('list');\n"
    "  list.innerHTML = '';\n"
    "  if(json.wifi.length==0 && json.ble.length==0){ list.innerHTML = '<i>No devices found yet</i>'; return; }\n"
    "  json.wifi.forEach(d=>{\n"
    "    const el = document.createElement('div'); el.className='device';\n"
    "    el.innerHTML = `<div><b>WIFI</b> ${d.ssid} <br/><small>${d.index}</small></div><div>rssi:${d.rssi} <br/> <button class='btn' onclick='select(\"wifi\",${d.index})'>Track</button></div>`;\n"
    "    list.appendChild(el);\n"
    "  });\n"
    "  json.ble.forEach(d=>{\n"
    "    const el = document.createElement('div'); el.className='device';\n"
    "    el.innerHTML = `<div><b>BLE</b> ${d.name} <br/><small>${d.mac}</small></div><div>rssi:${d.rssi} <br/> <button class='btn' onclick='select(\"ble\",${d.index})'>Track</button></div>`;\n"
    "    list.appendChild(el);\n"
    "  });\n"
    "}\n"
    "let pollInterval = null;\n"
    "async function select(type,index){\n"
    "  // send form-encoded POST\n"
    "  const body = new URLSearchParams(); body.append('type', type); body.append('index', String(index));\n"
    "  const respd = await fetch('/set_target', { method:'POST', headers: {'Content-Type': 'application/x-www-form-urlencoded'},body });\n"
    "  const disp = document.getElementById('distance');\n"
    "  //if(!respd.ok){ disp.innerText='Post was unsuccessfull'; return; }\n"
    "  // start polling distance\n"
    "  if(pollInterval) clearInterval(pollInterval);\n"
    
    "  pollInterval = setInterval(async()=>{\n"
    "    const resp = await fetch('/distance');\n"
    "    if(!resp.ok){ disp.innerText='No target set'; return; }\n"
    "    const j = await resp.json();\n"
    "    disp.innerText = `Type:${j.type} Index:${j.index} SSID:${j.ssid} Angle:${j.angle} Distance:${j.distance}`;\n"
    "  }, 1000);\n"
    "}\n"
    "// initial load and refresh every 3s\n"
    "fetchDevices(); setInterval(fetchDevices, 3000);\n"
    "  </script>\n"
    "</body>\n"
    "</html>\n";

/* / GET -> index */
static esp_err_t index_get_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, index_html, strlen(index_html));
    return ESP_OK;
}

/* /devices GET -> json of wifi and ble devices */
static esp_err_t devices_get_handler(httpd_req_t *req)
{
    // simple fixed-size buffer (adjust if you expect many devices)
    const int BUF_SZ = 8192;
    char *buf = malloc(BUF_SZ);
    if (!buf)
        return ESP_ERR_NO_MEM;
    int off = 0;
    off += snprintf(buf + off, BUF_SZ - off, "{ \"wifi\": [");
    for (int i = 0; i < g_wifi_count; ++i)
    {
        char mac[32];
        mac_to_str(wifi_devices[i].mac, mac, sizeof(mac));
        double dist = calculate_distance(wifi_devices[i]);
        off += snprintf(buf + off, BUF_SZ - off,
                        "{\"index\":%d,\"ssid\":\"%s\",\"mac\":\"%s\",\"rssi\":%ld,\"channel\":%u,\"distance\":%.2f}%s",
                        i, wifi_devices[i].ssid, mac, wifi_devices[i].smoothed_rssi, wifi_devices[i].channel,
                        dist, (i + 1 < g_wifi_count) ? "," : "");
        if (off > BUF_SZ - 200)
            break;
    }
    off += snprintf(buf + off, BUF_SZ - off, "], \"ble\": [");
    for (int j = 0; j < g_bt_count; ++j)
    {
        char mac[32];
        mac_to_str(bt_devices[j].mac, mac, sizeof(mac));
        double dist =calculate_distance2(bt_devices[j]);
        off += snprintf(buf + off, BUF_SZ - off,
                        "{\"index\":%d,\"name\":\"%s\",\"mac\":\"%s\",\"rssi\":%ld,\"distance\":%.2f}%s",
                        j, bt_devices[j].name, mac, bt_devices[j].smoothed_rssi, dist,
                        (j + 1 < g_bt_count) ? "," : "");
        if (off > BUF_SZ - 200)
            break;
    }
    off += snprintf(buf + off, BUF_SZ - off, "] }\n");
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, buf, off);
    free(buf);
    return ESP_OK;
}

/* /set_target POST -> form-urlencoded: type=wifi|ble & index=N
   sets target by calling set_target_device(...)
*/
static esp_err_t set_target_post_handler(httpd_req_t *req)
{
    int content_len = req->content_len;
    if (content_len <= 0 || content_len > 1024)
    {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "bad content length");
        return ESP_FAIL;
    }
    char *buf = malloc(content_len + 1);
    if (!buf)
        return ESP_ERR_NO_MEM;
    int ret = httpd_req_recv(req, buf, content_len);
    if (ret <= 0)
    {
        free(buf);
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "recv failed");
        return ESP_FAIL;
    }
    buf[ret] = '\0';
    ESP_LOGI(TAG, "POST body: %s", buf);
    // parse simple form "type=wifi&index=0"
    char *type = NULL;
    int index = -1;
    char *p = strstr(buf, "type=");
    char *t = strstr(buf, "index=");
    if (p)
    {
        p += 5;
        
        char *amp = strchr(p, '&');
        if (amp)
        {
            *amp = '\0';
            type = p;
        }
        else
            type = p;
        
    }
    //p = strstr(buf, "index=");
    if (t)
    {
        t += 6;
        index = atoi(t);
    }

    if (!type || index < 0)
    {
        free(buf);
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "missing type/index");
        return ESP_FAIL;
    }

    if (strcmp(type, "wifi") == 0)
    {
        if (index >= 0 && index < g_wifi_count)
        {
            // call set_target_device: (ssid ptr, channel, bssid ptr, ble ptr, direction)
            set_target_device((unsigned char *)wifi_devices[index].ssid,
                              wifi_devices[index].channel,
                              wifi_devices[index].mac,
                              NULL,
                              true);
            ESP_LOGI(TAG, "Target set to WIFI index %d SSID=%s", index, wifi_devices[index].ssid);
        }
        else
        {
            free(buf);
            httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "invalid wifi index");
            return ESP_FAIL;
        }
    }
    else if (strcmp(type, "ble") == 0)
    {
        if (index >= 0 && index < g_bt_count)
        {
            set_target_device(NULL, 0, NULL, bt_devices[index].mac, true);
            ESP_LOGI(TAG, "Target set to BLE index %d Name=%s", index, bt_devices[index].name);
        }
        else
        {
            free(buf);
            httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "invalid ble index");
            return ESP_FAIL;
        }
    }
    else
    {
        free(buf);
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "unknown type");
        return ESP_FAIL;
    }

    free(buf);
    httpd_resp_sendstr(req, "OK");
    return ESP_OK;
}

/* /distance GET -> returns JSON for currently set target (or query params if provided)
   optional query: ?type=wifi&index=0
*/
static esp_err_t distance_get_handler(httpd_req_t *req)
{
    char buf_q[128];
    size_t buf_len = httpd_req_get_url_query_len(req) + 1;

    if (buf_len > 1 && buf_len < sizeof(buf_q))
    {
        if (httpd_req_get_url_query_str(req, buf_q, buf_len) == ESP_OK)
        {
            // parse key=value
            char param[32];
            if (httpd_query_key_value(buf_q, "param", param, sizeof(param)) == ESP_OK)
            {
                ESP_LOGI(TAG, "Received param: %s", param);
            }
        }
    }
    char type[16] = {0};
    int index = -1;
    if (buf_q[0])
    {
        char *t = strstr(buf_q, "type=");
        if (t)
        {
            t += 5;
            sscanf(t, "%15[^&]", type);
        }
        char *i = strstr(buf_q, "index=");
        if (i)
        {
            i += 6;
            index = atoi(i);
        }
    }

    char resp[256];
    if (type[0])
    {
        if (strcmp(type, "wifi") == 0)
        {
            if (index < 0 || index >= g_wifi_count)
            {
                httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "invalid wifi index");
                return ESP_FAIL;
            }
            double dist = calculate_distance(wifi_devices[index]);
            int len = snprintf(resp, sizeof(resp), "{\"type\":\"wifi\",\"index\":%d,\"rssi\":%ld,\"distance\":%.2f}",
                               index, wifi_devices[index].smoothed_rssi, dist);
            httpd_resp_set_type(req, "application/json");
            httpd_resp_send(req, resp, len);
            return ESP_OK;
        }
        else if (strcmp(type, "ble") == 0)
        {
            if (index < 0 || index >= g_bt_count)
            {
                httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "invalid ble index");
                return ESP_FAIL;
            }
            int len = snprintf(resp, sizeof(resp), "{\"type\":\"ble\",\"index\":%d,\"rssi\":%ld,\"distance\":%.2f}",
                               index, bt_devices[index].smoothed_rssi, 0.0);
            httpd_resp_set_type(req, "application/json");
            httpd_resp_send(req, resp, len);
            return ESP_OK;
        }
        else
        {
            httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "unknown type");
            return ESP_FAIL;
        }
    } 

    // If no query, use current target if set
    if (target.is_direction)
    {
        //httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, " sdfshkdg target");
        ESP_LOGI(TAG, "Went inside direction cond");
        if (target.target_bssi)
        {
            // wifi target preferred if bssi set
            ESP_LOGI(TAG, "Went inside target_bssi cond");
            int idx = target.wifi_index;

            if (idx >= 0 && idx < g_wifi_count)
            {
                ESP_LOGI(TAG, "Went inside wifi index");
                //double dist = calculate_distance(wifi_devices[idx]);
                int best_angle = get_best_angle_for_target();
                double dist = calculate_distance(wifi_devices[index]);
                int len = snprintf(resp, sizeof(resp), "{\"type\":\"wifi\",\"index\":%d,\"ssid\":\"%s\",\"angle\":%d,\"distance\":%.2f}",
                                   idx,wifi_devices[index].ssid ,best_angle, dist);
                httpd_resp_set_type(req, "application/json");
                httpd_resp_send(req, resp, len);
                return ESP_OK;
            }
        }
        if (target.target_ble)
        {
            int idx = target.b_index;
            if (idx >= 0 && idx < g_bt_count)
            {
                
                int len = snprintf(resp, sizeof(resp), "{\"type\":\"ble\",\"index\":%d,\"rssi\":%ld,\"distance\":%.2f}",
                                   idx, bt_devices[idx].smoothed_rssi, 0.0);
                httpd_resp_set_type(req, "application/json");
                httpd_resp_send(req, resp, len);
                return ESP_OK;
            }
        }
    }

    httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "no jhrkgfdhfjshdgfhsdfshkdg target");
    return ESP_FAIL;
}

/* register URIs and start httpd */
void webserver_start()
{
    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();

    if (httpd_start(&server, &config) != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to start HTTP server");
        return;
    }

    httpd_uri_t idx = {
        .uri = "/",
        .method = HTTP_GET,
        .handler = index_get_handler,
        .user_ctx = NULL};
    httpd_register_uri_handler(server, &idx);

    httpd_uri_t dev = {
        .uri = "/devices",
        .method = HTTP_GET,
        .handler = devices_get_handler,
        .user_ctx = NULL};
    httpd_register_uri_handler(server, &dev);

    httpd_uri_t set = {
        .uri = "/set_target",
        .method = HTTP_POST,
        .handler = set_target_post_handler,
        .user_ctx = NULL};
    httpd_register_uri_handler(server, &set);

    httpd_uri_t dist = {
        .uri = "/distance",
        .method = HTTP_GET,
        .handler = distance_get_handler,
        .user_ctx = NULL};
    httpd_register_uri_handler(server, &dist);

    ESP_LOGI(TAG, "HTTP server started");
}
