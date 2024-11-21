#include "esp_camera.h"
#include <WiFi.h>
#include <WebServer.h>

// Camera model
#define CAMERA_MODEL_AI_THINKER
#define PWDN_GPIO_NUM  32
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM  0
#define SIOD_GPIO_NUM  26
#define SIOC_GPIO_NUM  27

#define Y9_GPIO_NUM    35
#define Y8_GPIO_NUM    34
#define Y7_GPIO_NUM    39
#define Y6_GPIO_NUM    36
#define Y5_GPIO_NUM    21
#define Y4_GPIO_NUM    19
#define Y3_GPIO_NUM    18
#define Y2_GPIO_NUM    5
#define VSYNC_GPIO_NUM 25
#define HREF_GPIO_NUM  23
#define PCLK_GPIO_NUM  22

// 4 for flash led or 33 for normal led
#define LED_GPIO_NUM   4

// WiFi credentials
const char *ssid = "PS";
const char *password = "p1r2i3y4a5";

// Create a web server object
WebServer server(80);

void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(false);
  Serial.println();

  // Camera configuration
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer   = LEDC_TIMER_0;
  config.pin_d0       = Y2_GPIO_NUM;
  config.pin_d1       = Y3_GPIO_NUM;
  config.pin_d2       = Y4_GPIO_NUM;
  config.pin_d3       = Y5_GPIO_NUM;
  config.pin_d4       = Y6_GPIO_NUM;
  config.pin_d5       = Y7_GPIO_NUM;
  config.pin_d6       = Y8_GPIO_NUM;
  config.pin_d7       = Y9_GPIO_NUM;
  config.pin_xclk     = XCLK_GPIO_NUM;
  config.pin_pclk     = PCLK_GPIO_NUM;
  config.pin_vsync    = VSYNC_GPIO_NUM;
  config.pin_href     = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn     = PWDN_GPIO_NUM;
  config.pin_reset    = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  config.frame_size   = FRAMESIZE_SVGA;
  config.jpeg_quality = 10;
  config.fb_count     = 2;

  // Camera initialization
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }

  // **Adjust Sensor Settings for Better Image Quality**
  sensor_t * s = esp_camera_sensor_get();
  s->set_brightness(s, 1);    // Increase brightness
  s->set_contrast(s, 1);      // Increase contrast
  s->set_saturation(s, 1);    // Increase saturation
  s->set_gainceiling(s, (gainceiling_t)0); // Set gain ceiling
  s->set_colorbar(s, 0);      // Disable colorbar test
  s->set_whitebal(s, 1);      // Enable white balance
  s->set_awb_gain(s, 1);      // Enable AWB gain
  s->set_wb_mode(s, 0);       // Auto WB mode
  s->set_exposure_ctrl(s, 1); // Enable exposure control
  s->set_aec2(s, 1);          // Enable 50/60Hz flicker reduction
  s->set_ae_level(s, 0);      // Exposure compensation
  s->set_aec_value(s, 300);   // Set AEC value
  s->set_gain_ctrl(s, 1);     // Enable gain control
  s->set_agc_gain(s, 0);      // AGC gain
  s->set_gainceiling(s, (gainceiling_t)0); // Gain ceiling
  s->set_bpc(s, 0);           // Disable black pixel correction
  s->set_wpc(s, 1);           // Enable white pixel correction
  s->set_raw_gma(s, 1);       // Enable gamma correction
  s->set_lenc(s, 1);          // Enable lens correction
  s->set_hmirror(s, 0);       // Disable horizontal mirror
  s->set_vflip(s, 0);         // Disable vertical flip
  s->set_dcw(s, 1);           // Enable downsize EN
  s->set_colorbar(s, 0);      // Disable colorbar test

  // Wi-Fi connection
  WiFi.begin(ssid, password);
  WiFi.setSleep(false);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
  }

  // Start the web server
  server.on("/", handleRoot);
  server.on("/image.jpg", handleImage);
  server.begin();

  Serial.print(WiFi.localIP());
}

void loop() {
  server.handleClient();
  if (Serial.available() > 0) {
    char command = Serial.read();
    if (command == 'R' || command == 'r') {
      Serial.println(WiFi.localIP());
    }
    // Clear any remaining input
    while (Serial.available() > 0) {
      Serial.read();
    }
  }

  delay(10); // Small delay to prevent high CPU usage
}

// Handler for the root path '/'
void handleRoot() {
  String html = "<html><head>";
  html += "<title>ESP32-CAM High Quality Image</title>";
  html += "<script type='text/javascript'>\n";
  html += "function reloadImage() {\n";
  html += "  var img = document.getElementById('camImage');\n";
  html += "  var timestamp = new Date().getTime();\n";
  html += "  img.src = '/image.jpg?t=' + timestamp;\n";
  html += "}\n";
  html += "setInterval(reloadImage, 1000); // Refresh every 1000 milliseconds (1 second)\n";
  html += "</script>\n";
  html += "</head><body>";
  html += "<img id='camImage' src='/image.jpg' />";
  html += "</body></html>";

  server.send(200, "text/html", html);
}

// Handler for '/image.jpg'
void handleImage() {
  camera_fb_t * fb = NULL;
  fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("Camera capture failed");
    server.send(500);
    return;
  }

  WiFiClient client = server.client();
  if (!client.connected()) {
    esp_camera_fb_return(fb);
    return;
  }

  // Send HTTP response headers
  client.println("HTTP/1.1 200 OK");
  client.println("Content-Type: image/jpeg");
  client.print("Content-Length: ");
  client.println(fb->len);
  client.println("Connection: close");
  client.println();

  // Send the image data
  client.write(fb->buf, fb->len);

  esp_camera_fb_return(fb);
}
