#include "esp_camera.h"
#include "esp_heap_caps.h"
#define ps_malloc(size) heap_caps_malloc(size, MALLOC_CAP_SPIRAM)

#include <WiFi.h>
#include "esp_timer.h"
#include "img_converters.h"
#include "esp_http_server.h"

#define CAMERA_MODEL_AI_THINKER
#include "camera_pins.h"

const char* ssid = "";     // Replace with your SSID
const char* password = ""; // Replace with your password

int latestX = -1, latestY = -1; // Centroid coordinates of detected cross

camera_config_t config;
httpd_handle_t stream_httpd = NULL;

// HTTP multipart stream headers for MJPEG
#define PART_BOUNDARY "123456789000000000000987654321"
static const char* _STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char* _STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char* _STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

#define FRAME_WIDTH 160
#define FRAME_HEIGHT 120

uint32_t* integral = NULL;  // Global integral image buffer (grayscale sum up to each pixel)

// Safely get pixel value from integral image with clamping
uint32_t get_integral(int x, int y) {
  if (x < 0) x = 0;
  if (y < 0) y = 0;
  if (x >= FRAME_WIDTH) x = FRAME_WIDTH - 1;
  if (y >= FRAME_HEIGHT) y = FRAME_HEIGHT - 1;
  return integral[y * FRAME_WIDTH + x];
}

// Compute sum of pixel values inside a box region using integral image
uint32_t box_sum(int x1, int y1, int x2, int y2) {
  return get_integral(x2, y2)
       - get_integral(x1 - 1, y2)
       - get_integral(x2, y1 - 1)
       + get_integral(x1 - 1, y1 - 1);
}

// Build the integral image from grayscale buffer
void compute_integral_image(uint8_t* buf, int width, int height) {
  for (int y = 0; y < height; y++) {
    uint32_t row_sum = 0;
    for (int x = 0; x < width; x++) {
      row_sum += buf[y * width + x];
      uint32_t above = (y > 0) ? integral[(y - 1) * width + x] : 0;
      integral[y * width + x] = row_sum + above;
    }
  }
}

// HTTP handler: captures, analyzes, annotates, and streams MJPEG frames
static esp_err_t stream_handler(httpd_req_t *req) {
  camera_fb_t *fb = NULL;
  char part_buf[64];
  esp_err_t res = httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);
  if (res != ESP_OK) return res;

  while (true) {
    fb = esp_camera_fb_get();
    if (!fb || fb->format != PIXFORMAT_GRAYSCALE) {
      if (fb) esp_camera_fb_return(fb);
      continue;
    }

    int width = fb->width;
    int height = fb->height;

    compute_integral_image(fb->buf, width, height);

    // Detect dark cross pattern by finding minimum horizontal + vertical sum
    int best_score = 999999999;
    int best_x = -1, best_y = -1;
    for (int y = 4; y < height - 4; y += 2) {
      for (int x = 4; x < width - 4; x += 2) {
        int h_sum = box_sum(x - 4, y, x + 4, y);
        int v_sum = box_sum(x, y - 4, x, y + 4);
        int score = h_sum + v_sum;
        if (score < best_score) {
          best_score = score;
          best_x = x;
          best_y = y;
        }
      }
    }

    latestX = best_x;
    latestY = best_y;

    if (latestX >= 0 && latestY >= 0) {
      // Draw white cross on the detected position
      for (int dx = -2; dx <= 2; dx++) {
        int px = latestX + dx;
        int py = latestY;
        if (px >= 0 && px < width) fb->buf[py * width + px] = 255;
      }
      for (int dy = -2; dy <= 2; dy++) {
        int px = latestX;
        int py = latestY + dy;
        if (py >= 0 && py < height) fb->buf[py * width + px] = 255;
      }

      Serial.print(latestX);
      Serial.print(",");
      Serial.println(latestY);
    }

    // Convert grayscale image to JPEG
    size_t jpg_buf_len = 0;
    uint8_t *jpg_buf = NULL;
    bool converted = frame2jpg(fb, 80, &jpg_buf, &jpg_buf_len);
    esp_camera_fb_return(fb);
    if (!converted) continue;

    // Stream JPEG frame over HTTP
    size_t hlen = snprintf(part_buf, 64, _STREAM_PART, jpg_buf_len);
    httpd_resp_send_chunk(req, part_buf, hlen);
    httpd_resp_send_chunk(req, (const char *)jpg_buf, jpg_buf_len);
    httpd_resp_send_chunk(req, _STREAM_BOUNDARY, strlen(_STREAM_BOUNDARY));

    free(jpg_buf);
    vTaskDelay(100 / portTICK_PERIOD_MS);  // ~10 FPS
  }

  return ESP_OK;
}

// Start HTTP server with single URI to stream MJPEG
void startCameraServer() {
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.server_port = 80;

  httpd_uri_t stream_uri = {
    .uri       = "/",
    .method    = HTTP_GET,
    .handler   = stream_handler,
    .user_ctx  = NULL
  };

  if (httpd_start(&stream_httpd, &config) == ESP_OK) {
    httpd_register_uri_handler(stream_httpd, &stream_uri);
  }
}

// Initialize camera, WiFi, and start streaming server
void setup() {
  Serial.begin(9600);

  // Camera pin and config setup
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_GRAYSCALE;
  config.frame_size = FRAMESIZE_QQVGA;  // 160x120
  config.jpeg_quality = 10;
  config.fb_count = 1;

  // Flash LED ON (optional)
  pinMode(4, OUTPUT);
  digitalWrite(4, HIGH);

  if (esp_camera_init(&config) != ESP_OK) {
    Serial.println("Camera init failed");
    return;
  }

  // Allocate memory for integral image
  integral = (uint32_t*)ps_malloc(FRAME_WIDTH * FRAME_HEIGHT * sizeof(uint32_t));
  if (!integral) {
    Serial.println("Failed to allocate integral buffer!");
    return;
  }

  // Connect to Wi-Fi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("\nWiFi connected");
  Serial.print("Stream available at: http://");
  Serial.println(WiFi.localIP());

  startCameraServer();
}

void loop() {
  delay(100);  // No logic here; all work done in streaming task
}
