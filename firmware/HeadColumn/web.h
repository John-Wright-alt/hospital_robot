#pragma once
// =====================================================================
//  web.h  -  the little web server on the ESP32.
//
//  Port 80:
//    GET /                 control page (live picture, buttons, status)
//    GET /status           JSON status  <- the ROS bridge reads this
//    GET /cmd?c=park       send a command (search, park, stop, home,
//    GET /cmd?c=up&v=10      wait, up, down, goto). Returns JSON.
//  Port 81:
//    GET /stream           live MJPEG video with the face box drawn on
//
//  Commands are put in a queue and carried out by the main loop, so the
//  web server never touches the motor directly.
// =====================================================================
#include <Arduino.h>
#include "esp_http_server.h"
#include "vision.h"

namespace web {

struct Command { char text[40]; };

static httpd_handle_t s_mainServer = nullptr;
static httpd_handle_t s_streamServer = nullptr;
static QueueHandle_t  s_cmdQueue = nullptr;
static char           s_status[768] = "{}";
static portMUX_TYPE   s_statusMux = portMUX_INITIALIZER_UNLOCKED;

static const char INDEX_HTML[] = R"HTML(<!doctype html>
<html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Head Column</title>
<style>
body{font-family:system-ui,sans-serif;margin:0;background:#15171a;color:#e8e8e8}
main{max-width:700px;margin:auto;padding:12px}
h2{margin:6px 0 10px}
img{width:100%;max-width:640px;aspect-ratio:4/3;background:#000;border-radius:8px;display:block}
.row{margin:10px 0}
button{font-size:16px;padding:10px 14px;margin:3px;border:0;border-radius:8px;background:#2f7d57;color:#fff;cursor:pointer}
button.b{background:#35597f} button.r{background:#b33636;font-weight:bold}
#big{font-size:20px;margin:8px 0}
pre{background:#22262b;padding:10px;border-radius:8px;font-size:13px;overflow-x:auto}
</style></head><body><main>
<h2>Head Column</h2>
<img id="cam" alt="camera stream (camera off or starting)">
<div id="big">...</div>
<div class="row">
<button onclick="cmd('search')">Search / lift</button>
<button class="b" onclick="cmd('park')">Park (lower)</button>
<button class="r" onclick="cmd('stop')">STOP</button>
</div><div class="row">
<button class="b" onclick="cmd('home')">Home</button>
<button class="b" onclick="cmd('up','10')">Up 10 mm</button>
<button class="b" onclick="cmd('down','10')">Down 10 mm</button>
<button class="b" onclick="cmd('wait')">Hold</button>
</div>
<pre id="st">loading...</pre>
</main><script>
document.getElementById('cam').src='http://'+location.hostname+':81/stream';
function cmd(c,v){fetch('/cmd?c='+c+(v?'&v='+v:'')).catch(e=>{});}
async function poll(){
 try{const r=await fetch('/status');const j=await r.json();
  document.getElementById('big').textContent=j.state+'  |  height '+j.height_mm.toFixed(1)+' mm  |  face: '+(j.face?'YES':'no')+'  |  '+j.fps.toFixed(1)+' fps';
  document.getElementById('st').textContent=JSON.stringify(j,null,1);
 }catch(e){document.getElementById('big').textContent='offline';}
 setTimeout(poll,400);}
poll();
</script></body></html>)HTML";

// ---- helpers for the main program ----
inline void setStatus(const char* json) {
  portENTER_CRITICAL(&s_statusMux);
  strlcpy(s_status, json, sizeof(s_status));
  portEXIT_CRITICAL(&s_statusMux);
}

inline bool popCommand(char* out, size_t n) {
  if (!s_cmdQueue) return false;
  Command c;
  if (xQueueReceive(s_cmdQueue, &c, 0) != pdTRUE) return false;
  strlcpy(out, c.text, n);
  return true;
}

// keep only safe characters (letters, digits, '.', '-')
static void sanitize(char* s) {
  char* w = s;
  for (char* r = s; *r; r++) {
    char ch = *r;
    if (ch >= 'A' && ch <= 'Z') ch = ch - 'A' + 'a';
    if ((ch >= 'a' && ch <= 'z') || (ch >= '0' && ch <= '9') || ch == '.' || ch == '-') *w++ = ch;
  }
  *w = 0;
}

// ---- request handlers ----
static esp_err_t indexHandler(httpd_req_t* req) {
  httpd_resp_set_type(req, "text/html");
  return httpd_resp_send(req, INDEX_HTML, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t statusHandler(httpd_req_t* req) {
  char buf[sizeof(s_status)];
  portENTER_CRITICAL(&s_statusMux);
  strlcpy(buf, s_status, sizeof(buf));
  portEXIT_CRITICAL(&s_statusMux);
  httpd_resp_set_type(req, "application/json");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  httpd_resp_set_hdr(req, "Cache-Control", "no-store");
  return httpd_resp_send(req, buf, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t cmdHandler(httpd_req_t* req) {
  char query[96] = {0}, c[24] = {0}, v[16] = {0};
  if (httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK) {
    httpd_query_key_value(query, "c", c, sizeof(c));
    httpd_query_key_value(query, "v", v, sizeof(v));
  }
  sanitize(c);
  sanitize(v);
  Command cmd;
  if (v[0]) snprintf(cmd.text, sizeof(cmd.text), "%s %s", c, v);
  else      snprintf(cmd.text, sizeof(cmd.text), "%s", c);
  bool ok = c[0] && s_cmdQueue && xQueueSend(s_cmdQueue, &cmd, 0) == pdTRUE;
  char resp[96];
  snprintf(resp, sizeof(resp), "{\"ok\":%s,\"command\":\"%s\"}", ok ? "true" : "false", cmd.text);
  httpd_resp_set_type(req, "application/json");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  return httpd_resp_send(req, resp, HTTPD_RESP_USE_STRLEN);
}

#define PART_BOUNDARY "headcolumnframe"
static const char* STREAM_TYPE     = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char* STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char* STREAM_PART     = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

static esp_err_t streamHandler(httpd_req_t* req) {
  if (!vision::cameraOk()) {
    httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "camera not running");
    return ESP_FAIL;
  }
  esp_err_t res = httpd_resp_set_type(req, STREAM_TYPE);
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  vision::s_streamClients = vision::s_streamClients + 1;

  uint8_t* local = nullptr;
  size_t cap = 0;
  uint32_t lastSeq = vision::s_jpgSeq;     // wait for a fresh frame
  uint32_t lastFrameMs = millis();

  while (res == ESP_OK) {
    size_t len = 0;
    if (xSemaphoreTake(vision::s_jpgMutex, pdMS_TO_TICKS(200)) == pdTRUE) {
      if (vision::s_jpg && vision::s_jpgSeq != lastSeq) {
        if (vision::s_jpgLen > cap) {
          free(local);
          cap = vision::s_jpgLen + 4096;
          local = (uint8_t*)malloc(cap);
          if (!local) cap = 0;
        }
        if (local) {
          memcpy(local, vision::s_jpg, vision::s_jpgLen);
          len = vision::s_jpgLen;
          lastSeq = vision::s_jpgSeq;
        }
      }
      xSemaphoreGive(vision::s_jpgMutex);
    }
    if (len == 0) {
      if (millis() - lastFrameMs > 5000) break;   // no pictures for 5 s: give up
      vTaskDelay(pdMS_TO_TICKS(15));
      continue;
    }
    lastFrameMs = millis();
    char part[80];
    int hlen = snprintf(part, sizeof(part), STREAM_PART, (unsigned)len);
    res = httpd_resp_send_chunk(req, STREAM_BOUNDARY, strlen(STREAM_BOUNDARY));
    if (res == ESP_OK) res = httpd_resp_send_chunk(req, part, hlen);
    if (res == ESP_OK) res = httpd_resp_send_chunk(req, (const char*)local, len);
  }

  vision::s_streamClients = vision::s_streamClients - 1;
  free(local);
  return res;
}

// ---- start both servers ----
inline void begin() {
  s_cmdQueue = xQueueCreate(8, sizeof(Command));

  httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
  cfg.server_port = 80;
  cfg.ctrl_port = 32768;
  cfg.max_uri_handlers = 8;
  cfg.stack_size = 6144;
  if (httpd_start(&s_mainServer, &cfg) == ESP_OK) {
    httpd_uri_t u = {};
    u.method = HTTP_GET;
    u.uri = "/";        u.handler = indexHandler;  httpd_register_uri_handler(s_mainServer, &u);
    u.uri = "/status";  u.handler = statusHandler; httpd_register_uri_handler(s_mainServer, &u);
    u.uri = "/cmd";     u.handler = cmdHandler;    httpd_register_uri_handler(s_mainServer, &u);
    Serial.println("[web] control page + API on port 80");
  } else {
    Serial.println("[web] FAILED to start server on port 80");
  }

  cfg.server_port = 81;
  cfg.ctrl_port = 32769;
  if (httpd_start(&s_streamServer, &cfg) == ESP_OK) {
    httpd_uri_t u = {};
    u.method = HTTP_GET;
    u.uri = "/stream";  u.handler = streamHandler; httpd_register_uri_handler(s_streamServer, &u);
    Serial.println("[web] video stream on port 81");
  }
}

}  // namespace web
