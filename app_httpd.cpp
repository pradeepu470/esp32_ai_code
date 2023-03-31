#include "esp_http_server.h"
#include "Preferences.h"
#include "WiFi.h"
#include "esp_timer.h"
#include "esp_camera.h"
#include "img_converters.h"
#include "camera_index.h"
#include "Arduino.h"
#include <netinet/in.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>
#include <EEPROM.h>
#include <Preferences.h>
#include "FS.h"
#include "SD_MMC.h"
#include "fb_gfx.h"
#include "fd_forward.h"
#include "fr_forward.h"
#include <pthread.h>

#define ENROLL_CONFIRM_TIMES 5
#define FACE_ID_SAVE_NUMBER 7
#define PORT 8080
#define PORT1 7240 // socket port


#define FACE_COLOR_WHITE 0x00FFFFFF
#define FACE_COLOR_BLACK 0x00000000
#define FACE_COLOR_RED 0x000000FF
#define FACE_COLOR_GREEN 0x0000FF00
#define FACE_COLOR_BLUE 0x00FF0000
#define FACE_COLOR_YELLOW (FACE_COLOR_RED | FACE_COLOR_GREEN)
#define FACE_COLOR_CYAN (FACE_COLOR_BLUE | FACE_COLOR_GREEN)
#define FACE_COLOR_PURPLE (FACE_COLOR_BLUE | FACE_COLOR_RED)
const char *wifi_ssid = "ESPCAM32";
void startCameraServer();
void Clientcreation(String sending_msg);
static int readFile();
Preferences preferences;
String sending_msg;


typedef struct {
  size_t size;   //number of values used for filtering
  size_t index;  //current value index
  size_t count;  //value count
  int sum;
  int *values;  //array to be filled with values
} ra_filter_t;

typedef struct {
  httpd_req_t *req;
  size_t len;
} jpg_chunking_t;

#define PART_BOUNDARY "123456789000000000000987654321"
static const char *_STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char *_STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char *_STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

static ra_filter_t ra_filter;
httpd_handle_t stream_httpd = NULL;
httpd_handle_t camera_httpd = NULL;

static mtmn_config_t mtmn_config = { 0 };
static int8_t detection_enabled = 1;
static int8_t recognition_enabled = 1;
static int8_t is_enrolling = 0;
static face_id_list id_list = {};


int enroll_id = 0;
int temp_var = 1;
int sample_id = 0;

WiFiServer server(23);


static ra_filter_t *ra_filter_init(ra_filter_t *filter, size_t sample_size) {
  memset(filter, 0, sizeof(ra_filter_t));

  filter->values = (int *)malloc(sample_size * sizeof(int)) ;
  if (!filter->values) {
    return NULL;
  }
  memset(filter->values, 0, sample_size * sizeof(int));

  filter->size = sample_size;
  return filter;
}

static int ra_filter_run(ra_filter_t *filter, int value) {
  if (!filter->values) {
    return value;
  }
  filter->sum -= filter->values[filter->index];
  filter->values[filter->index] = value;
  filter->sum += filter->values[filter->index];
  filter->index++;
  filter->index = filter->index % filter->size;
  if (filter->count < filter->size) {
    filter->count++;
  }
  return filter->sum / filter->count;
}

static void rgb_print(dl_matrix3du_t *image_matrix, uint32_t color, const char *str) {
  fb_data_t fb;
  fb.width = image_matrix->w;
  fb.height = image_matrix->h;
  fb.data = image_matrix->item;
  fb.bytes_per_pixel = 3;
  fb.format = FB_BGR888;
  fb_gfx_print(&fb, (fb.width - (strlen(str) * 14)) / 2, 10, color, str);
}

static int rgb_printf(dl_matrix3du_t *image_matrix, uint32_t color, const char *format, ...) {
  char loc_buf[64];
  char *temp = loc_buf;
  int len;
  va_list arg;
  va_list copy;
  va_start(arg, format);
  va_copy(copy, arg);
  len = vsnprintf(loc_buf, sizeof(loc_buf), format, arg);
  va_end(copy);
  if (len >= sizeof(loc_buf)) {
    temp = (char *)malloc(len + 1);
    if (temp == NULL) {
      return 0;
    }
  }
  vsnprintf(temp, len + 1, format, arg);
  va_end(arg);
  rgb_print(image_matrix, color, temp);
  if (len > 64) {
    free(temp);
  }
  return len;
}

static void draw_face_boxes(dl_matrix3du_t *image_matrix, box_array_t *boxes, int face_id) {
  int x, y, w, h, i;
  uint32_t color = FACE_COLOR_YELLOW;
  if (face_id < 0) {
    color = FACE_COLOR_RED;
  } else if (face_id > 0) {
    color = FACE_COLOR_GREEN;
  }
  fb_data_t fb;
  fb.width = image_matrix->w;
  fb.height = image_matrix->h;
  fb.data = image_matrix->item;
  fb.bytes_per_pixel = 3;
  fb.format = FB_BGR888;
  for (i = 0; i < boxes->len; i++) {
    // rectangle box
    x = (int)boxes->box[i].box_p[0];
    y = (int)boxes->box[i].box_p[1];
    w = (int)boxes->box[i].box_p[2] - x + 1;
    h = (int)boxes->box[i].box_p[3] - y + 1;
    fb_gfx_drawFastHLine(&fb, x, y, w, color);
    fb_gfx_drawFastHLine(&fb, x, y + h - 1, w, color);
    fb_gfx_drawFastVLine(&fb, x, y, h, color);
    fb_gfx_drawFastVLine(&fb, x + w - 1, y, h, color);
#if 0
        // landmark
        int x0, y0, j;
        for (j = 0; j < 10; j+=2) {
            x0 = (int)boxes->landmark[i].landmark_p[j];
            y0 = (int)boxes->landmark[i].landmark_p[j+1];
            fb_gfx_fillRect(&fb, x0, y0, 3, 3, color);
        }
#endif
  }
}


// below function is used for read files from sdcard

// this function will read images from SD card and enroll images into ESP32


void SD_ENROLL(){

    mtmn_config = mtmn_init_config();
    
    face_id_init(&id_list, FACE_ID_SAVE_NUMBER, ENROLL_CONFIRM_TIMES);

    dl_matrix3du_t *aligned_face = NULL;
    int8_t left_sample_face = NULL;
    dl_matrix3du_t *image_matrix = NULL;
      
    if(!SD_MMC.begin()){
        Serial.println("Card Mount Failed");        
    }
    fs::FS &fs = SD_MMC;
    uint8_t cardType = SD_MMC.cardType();
    if(cardType == CARD_NONE){
        Serial.println("No SD_MMC card attached");       
    }

  for (int i=0;i<6;i++){
    Serial.printf("Enrolling SD card Face ID: %d ", i);
    for (int j=1;j<6;j++){
        String sd_path = "/picture" +String(i)+"_"+String(j) + ".jpg";
        Serial.println(sd_path);
        File file = fs.open(sd_path);
        if(!file){
            Serial.println("Failed to open file for reading");
            break;
        }else {
        char *buf;
        buf = (char*) malloc (sizeof(char)*file.size());
        long i = 0;
        while (file.available()) {
          buf[i] = file.read(); 
          i++;  
        }
        int image_width = 320;
        int image_height = 240;
        image_matrix = dl_matrix3du_alloc(1, image_width, image_height, 3);
        if (!image_matrix) {
            Serial.println("dl_matrix3du_alloc failed");
        } else {          
            fmt2rgb888((uint8_t*)buf, file.size(), PIXFORMAT_JPEG, image_matrix->item);
            box_array_t *net_boxes = face_detect(image_matrix, &mtmn_config);
            if (net_boxes){                    
              aligned_face = dl_matrix3du_alloc(1, FACE_WIDTH, FACE_HEIGHT, 3);
              if (align_face(net_boxes, image_matrix, aligned_face) == ESP_OK){
                  if(!aligned_face){
                      Serial.println("Could not allocate face recognition buffer");
                  }else {
                      int8_t left_sample_face = enroll_face(&id_list, aligned_face);
                      Serial.println(left_sample_face);
                      if(left_sample_face == (ENROLL_CONFIRM_TIMES - 1)){
                          enroll_id = id_list.tail;
                          Serial.printf("Enrolling Face ID: %d\n", enroll_id); 
                          
                      }
                     Serial.printf("Enrolling SD card Face ID: %d sample %d\n", enroll_id, ENROLL_CONFIRM_TIMES - left_sample_face);
                      if (left_sample_face == 0){                         
                          Serial.printf("Enrolled Face ID: %d\n", enroll_id);
                      }
                      Serial.println();
                    }
                  dl_matrix3du_free(aligned_face);
                }              
              dl_lib_free(net_boxes->score);
              dl_lib_free(net_boxes->box);
              dl_lib_free(net_boxes->landmark);
              dl_lib_free(net_boxes);                                
              net_boxes = NULL;
            }
            else {
              Serial.println("No Face");
              Serial.println();
            }
            dl_matrix3du_free(image_matrix);
        }
        free(buf);
      }
    }
  }
}

// the function is ended here




// this funciton is for recognise the face
// if enroll is enabled it enroll the face 
//  if enroll is disabled it will go for matching face ID's 

static int run_face_recognition(dl_matrix3du_t *image_matrix, box_array_t *net_boxes,camera_fb_t *fb) {

  dl_matrix3du_t *aligned_face = NULL;
  int matched_id = 0;
  int sd_matched_id = 0;

  aligned_face = dl_matrix3du_alloc(1, FACE_WIDTH, FACE_HEIGHT, 3);
  if (!aligned_face) {
    Serial.println("Could not allocate face recognition buffer");
    return matched_id;
  }
  if (align_face(net_boxes, image_matrix, aligned_face) == ESP_OK) {
   
      if (is_enrolling == 1) {

        int8_t left_sample_face = enroll_face(&id_list, aligned_face);
        
        Serial.println(left_sample_face);

        if (left_sample_face == (ENROLL_CONFIRM_TIMES - 1)) {
          enroll_id = id_list.tail;
          Serial.printf("Enrolling Face ID: %d\n", enroll_id);
        }
        Serial.printf("Enrolling Golden Face ID: %d sample %d\n", enroll_id, ENROLL_CONFIRM_TIMES - left_sample_face);
        rgb_printf(image_matrix, FACE_COLOR_CYAN, "ID[%u] Sample[%u]", id_list.tail, ENROLL_CONFIRM_TIMES - left_sample_face);

        if (left_sample_face == 0) {
          is_enrolling = 0;
          Serial.printf("Enrolled success for Face ID: %d\n", enroll_id);
        }   


        SD_MMC.begin("/sdcard", true); // from here i am storing the images in SD card
        if (!SD_MMC.begin()) {
          Serial.println("SD Card Mount Failed");
        }
        uint8_t cardType = SD_MMC.cardType();
        if (cardType == CARD_NONE) {
          Serial.println("No SD Card attached");
        }

          sample_id = temp_var;
          rgb_printf(image_matrix, FACE_COLOR_CYAN, "ID[%u] Sample[%u]", enroll_id, sample_id);
          String path = "/picture" + String(enroll_id)+"_"+String(sample_id) + ".jpg";

          fs::FS &fs = SD_MMC;

          File file = fs.open(path.c_str(), FILE_WRITE);
          if (!file) {
            Serial.println("Failed to open file in writing mode");
          } else {
            file.write(fb->buf, fb->len);  // payload (image), payload length
            Serial.printf("Saved file to path: %s\n", path.c_str());
          }
          file.close();
        temp_var = sample_id + 1;
        if (temp_var == 6){
          temp_var = 1;
        }

    } 
    else {

      
      matched_id = recognize_face(&id_list, aligned_face);
      if (matched_id >= 0) {
        Serial.printf("Match Face ID: %u\n", matched_id);
        rgb_printf(image_matrix, FACE_COLOR_GREEN, "Hello Subject %u", matched_id);
        sending_msg = "FBoxAIData:Success,Match Face ID:"+String(matched_id);
        //Clientcreation(sending_msg);
      } else {  

        Serial.println("No Match Found");
        rgb_print(image_matrix, FACE_COLOR_RED, "Intruder Alert!");
        matched_id = -1;
        }
      }
    
  } else {
    Serial.println("Face Not Aligned");
    //rgb_print(image_matrix, FACE_COLOR_YELLOW, "Human Detected");
  }

  dl_matrix3du_free(aligned_face);
  return matched_id;
}


// conversion of frame to jpg
static size_t jpg_encode_stream(void *arg, size_t index, const void *data, size_t len) {
  jpg_chunking_t *j = (jpg_chunking_t *)arg;
  if (!index) {
    j->len = 0;
  }
  if (httpd_resp_send_chunk(j->req, (const char *)data, len) != ESP_OK) {
    return 0;
  }
  j->len += len;
  return len;
}

// screenshot taking(Get still).. we dont need this function

static esp_err_t capture_handler(httpd_req_t *req) {
  camera_fb_t *fb = NULL;
  esp_err_t res = ESP_OK;
  int64_t fr_start = esp_timer_get_time();
  Serial.println("capture_handler");

  fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("Camera capture failed");
    httpd_resp_send_500(req);
    return ESP_FAIL;
  }

  httpd_resp_set_type(req, "image/jpeg");
  httpd_resp_set_hdr(req, "Content-Disposition", "inline; filename=capture.jpg");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

  size_t out_len, out_width, out_height;
  uint8_t *out_buf;
  bool s;
  bool detected = false;
  int face_id = 0;
  if (!detection_enabled || fb->width > 400) {
    size_t fb_len = 0;
    if (fb->format == PIXFORMAT_JPEG) {
      fb_len = fb->len;
      res = httpd_resp_send(req, (const char *)fb->buf, fb->len);
    } else {
      jpg_chunking_t jchunk = { req, 0 };
      res = frame2jpg_cb(fb, 80, jpg_encode_stream, &jchunk) ? ESP_OK : ESP_FAIL;
      httpd_resp_send_chunk(req, NULL, 0);
      fb_len = jchunk.len;
    }
    esp_camera_fb_return(fb);
    int64_t fr_end = esp_timer_get_time();
    //Serial.printf("JPG: %uB %ums\n", (uint32_t)(fb_len), (uint32_t)((fr_end - fr_start)/1000));
    return res;
  }

  dl_matrix3du_t *image_matrix = dl_matrix3du_alloc(1, fb->width, fb->height, 3);
  if (!image_matrix) {
    esp_camera_fb_return(fb);
    Serial.println("dl_matrix3du_alloc failed");
    httpd_resp_send_500(req);
    return ESP_FAIL;
  }

  out_buf = image_matrix->item;
  out_len = fb->width * fb->height * 3;
  out_width = fb->width;
  out_height = fb->height;

  s = fmt2rgb888(fb->buf, fb->len, fb->format, out_buf);
  esp_camera_fb_return(fb);
  if (!s) {
    dl_matrix3du_free(image_matrix);
    Serial.println("to rgb888 failed");
    httpd_resp_send_500(req);
    return ESP_FAIL;
  }

  box_array_t *net_boxes = face_detect(image_matrix, &mtmn_config);

  if (net_boxes) {
    detected = true;
    if (recognition_enabled) {
      face_id = run_face_recognition(image_matrix, net_boxes,fb);
    }
    draw_face_boxes(image_matrix, net_boxes, face_id);
    dl_lib_free(net_boxes->score);
    dl_lib_free(net_boxes->box);
    dl_lib_free(net_boxes->landmark);
    dl_lib_free(net_boxes);
  }

  jpg_chunking_t jchunk = { req, 0 };
  s = fmt2jpg_cb(out_buf, out_len, out_width, out_height, PIXFORMAT_RGB888, 90, jpg_encode_stream, &jchunk);
  dl_matrix3du_free(image_matrix);
  if (!s) {
    Serial.println("JPEG compression failed");
    return ESP_FAIL;
  }

  int64_t fr_end = esp_timer_get_time();
  Serial.printf("FACE: %uB %ums %s%d\n", (uint32_t)(jchunk.len), (uint32_t)((fr_end - fr_start) / 1000), detected ? "DETECTED " : "", face_id);
  return res;
}


// streaming video
static esp_err_t stream_handler(httpd_req_t *req) {
  //Serial.printf("stream uri");
  camera_fb_t *fb = NULL;
  esp_err_t res = ESP_OK;
  size_t _jpg_buf_len = 0;
  uint8_t *_jpg_buf = NULL;
  char *part_buf[64];
  dl_matrix3du_t *image_matrix = NULL;
  bool detected = false;
  int face_id = 0;
  int64_t fr_start = 0;
  int64_t fr_ready = 0;
  int64_t fr_face = 0;
  int64_t fr_recognize = 0;
  int64_t fr_encode = 0;

  static int64_t last_frame = 0;
  if (!last_frame) {
    last_frame = esp_timer_get_time();
  }

  res = httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);
  if (res != ESP_OK) {
    return res;
  }

  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

  while (true) {
    detected = false;
    face_id = 0;
    fb = esp_camera_fb_get();
    if (!fb) {
      Serial.println("Camera capture failed");
      res = ESP_FAIL;
    } else {
      fr_start = esp_timer_get_time();
      fr_ready = fr_start;
      fr_face = fr_start;
      fr_encode = fr_start;
      fr_recognize = fr_start;
      enroll_id = id_list.tail;
      //if(!detection_enabled || fb->width > 400){
      if (fb->format != PIXFORMAT_JPEG) {
        bool jpeg_converted = frame2jpg(fb, 80, &_jpg_buf, &_jpg_buf_len);
        esp_camera_fb_return(fb);
        fb = NULL;
        if (!jpeg_converted) {
          Serial.println("JPEG compression failed");
          res = ESP_FAIL;
        }
      } else {
        _jpg_buf_len = fb->len;
        _jpg_buf = fb->buf;
      }
      //} else {

      image_matrix = dl_matrix3du_alloc(1, fb->width, fb->height, 3);  //swa

      if (!image_matrix) {
        Serial.println("dl_matrix3du_alloc failed");
        res = ESP_FAIL;
      } else {
        // if (enroll_id = 0) {
        if (!fmt2rgb888(fb->buf, fb->len, fb->format, image_matrix->item)) {
          Serial.println("fmt2rgb888 failed");
          res = ESP_FAIL;
        // }
        } else {
          fr_ready = esp_timer_get_time();
          box_array_t *net_boxes = NULL;
          if (detection_enabled) {
            net_boxes = face_detect(image_matrix, &mtmn_config);
          }
          fr_face = esp_timer_get_time();
          fr_recognize = fr_face;
          if (net_boxes || fb->format != PIXFORMAT_JPEG) {
            if (net_boxes) {
              detected = true;
              if (recognition_enabled) {
                Serial.println("stream handler");
                face_id = run_face_recognition(image_matrix, net_boxes, fb);
              }
              fr_recognize = esp_timer_get_time();
              draw_face_boxes(image_matrix, net_boxes, face_id);
              dl_lib_free(net_boxes->score);
              dl_lib_free(net_boxes->box);
              dl_lib_free(net_boxes->landmark);
              dl_lib_free(net_boxes);
            }
            // if (enroll_id = 0){
            if (!fmt2jpg(image_matrix->item, fb->width * fb->height * 8, fb->width, fb->height, PIXFORMAT_RGB888, 90, &_jpg_buf, &_jpg_buf_len)) {
                Serial.println("fmt2jpg failed");
                res = ESP_FAIL;
            }
            // }
            esp_camera_fb_return(fb);
            fb = NULL;
          } else {
            _jpg_buf = fb->buf;
            _jpg_buf_len = fb->len;
          }
          fr_encode = esp_timer_get_time();
        }
        dl_matrix3du_free(image_matrix);
      }
      //}
    }
    //Serial.println("stream sending...");
    if (res == ESP_OK) {
      res = httpd_resp_send_chunk(req, _STREAM_BOUNDARY, strlen(_STREAM_BOUNDARY));
    }
    if (res == ESP_OK) {
      size_t hlen = snprintf((char *)part_buf, 64, _STREAM_PART, _jpg_buf_len);
      res = httpd_resp_send_chunk(req, (const char *)part_buf, hlen);
    }
    if (res == ESP_OK) {
      res = httpd_resp_send_chunk(req, (const char *)_jpg_buf, _jpg_buf_len);
    }
    if (fb) {
      esp_camera_fb_return(fb);
      fb = NULL;
      _jpg_buf = NULL;
    } else if (_jpg_buf) {
      free(_jpg_buf);
      _jpg_buf = NULL;
    }
    if (res != ESP_OK) {
      break;
    }
  }

  last_frame = 0;
  return res;
}

// every commands sent come here..enroll, requests etc

static esp_err_t cmd_handler(httpd_req_t *req) {
  char *buf;
  size_t buf_len;
  char variable[32] = {
    0,
  };
  char value[32] = {
    0,
  };

  buf_len = httpd_req_get_url_query_len(req) + 1;
  if (buf_len > 1) {
    buf = (char *)malloc(buf_len);
    if (!buf) {
      httpd_resp_send_500(req);
      return ESP_FAIL;
    }
    if (httpd_req_get_url_query_str(req, buf, buf_len) == ESP_OK) {
      if (httpd_query_key_value(buf, "var", variable, sizeof(variable)) == ESP_OK && httpd_query_key_value(buf, "val", value, sizeof(value)) == ESP_OK) {
      } else {
        free(buf);
        httpd_resp_send_404(req);
        return ESP_FAIL;
      }
    } else {
      free(buf);
      httpd_resp_send_404(req);
      return ESP_FAIL;
    }
    free(buf);
  } else {
    httpd_resp_send_404(req);
    return ESP_FAIL;
  }

  int val = atoi(value);
  sensor_t *s = esp_camera_sensor_get();
  int res = 0;
  Serial.printf(variable);

  if (!strcmp(variable, "framesize")) {
    if (s->pixformat == PIXFORMAT_JPEG) res = s->set_framesize(s, (framesize_t)val);
  } else if (!strcmp(variable, "quality")) res = s->set_quality(s, val);
  else if (!strcmp(variable, "contrast")) res = s->set_contrast(s, val);
  else if (!strcmp(variable, "brightness")) res = s->set_brightness(s, val);
  else if (!strcmp(variable, "saturation")) res = s->set_saturation(s, val);
  else if (!strcmp(variable, "gainceiling")) res = s->set_gainceiling(s, (gainceiling_t)val);
  else if (!strcmp(variable, "colorbar")) res = s->set_colorbar(s, val);
  else if (!strcmp(variable, "awb")) res = s->set_whitebal(s, val);
  else if (!strcmp(variable, "agc")) res = s->set_gain_ctrl(s, val);
  else if (!strcmp(variable, "aec")) res = s->set_exposure_ctrl(s, val);
  else if (!strcmp(variable, "hmirror")) res = s->set_hmirror(s, val);
  else if (!strcmp(variable, "vflip")) res = s->set_vflip(s, val);
  else if (!strcmp(variable, "awb_gain")) res = s->set_awb_gain(s, val);
  else if (!strcmp(variable, "agc_gain")) res = s->set_agc_gain(s, val);
  else if (!strcmp(variable, "aec_value")) res = s->set_aec_value(s, val);
  else if (!strcmp(variable, "aec2")) res = s->set_aec2(s, val);
  else if (!strcmp(variable, "dcw")) res = s->set_dcw(s, val);
  else if (!strcmp(variable, "bpc")) res = s->set_bpc(s, val);
  else if (!strcmp(variable, "wpc")) res = s->set_wpc(s, val);
  else if (!strcmp(variable, "raw_gma")) res = s->set_raw_gma(s, val);
  else if (!strcmp(variable, "lenc")) res = s->set_lenc(s, val);
  else if (!strcmp(variable, "special_effect")) res = s->set_special_effect(s, val);
  else if (!strcmp(variable, "wb_mode")) res = s->set_wb_mode(s, val);
  else if (!strcmp(variable, "ae_level")) res = s->set_ae_level(s, val);
  else if (!strcmp(variable, "face_detect")) {
    // Serial.println("i am here at face_detect");
    detection_enabled = val;
    if (!detection_enabled) {
      recognition_enabled = 0;
    }
  } else if (!strcmp(variable, "face_enroll")) is_enrolling = val;
  else if (!strcmp(variable, "face_recognize")) {
    recognition_enabled = val;
    if (recognition_enabled) {
      detection_enabled = val;
    }
  } else {
    res = -1;
  }
  if (res) {
    return httpd_resp_send_500(req);
  }
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  return httpd_resp_send(req, NULL, 0);
}

static esp_err_t status_handler(httpd_req_t *req) {
  static char json_response[1024];

  sensor_t *s = esp_camera_sensor_get();
  char *p = json_response;
  *p++ = '{';
  p += sprintf(p, "\"framesize\":%u,", s->status.framesize);
  p += sprintf(p, "\"quality\":%u,", s->status.quality);
  p += sprintf(p, "\"brightness\":%d,", s->status.brightness);
  p += sprintf(p, "\"contrast\":%d,", s->status.contrast);
  p += sprintf(p, "\"saturation\":%d,", s->status.saturation);
  p += sprintf(p, "\"sharpness\":%d,", s->status.sharpness);
  p += sprintf(p, "\"special_effect\":%u,", s->status.special_effect);
  p += sprintf(p, "\"wb_mode\":%u,", s->status.wb_mode);
  p += sprintf(p, "\"awb\":%u,", s->status.awb);
  p += sprintf(p, "\"awb_gain\":%u,", s->status.awb_gain);
  p += sprintf(p, "\"aec\":%u,", s->status.aec);
  p += sprintf(p, "\"aec2\":%u,", s->status.aec2);
  p += sprintf(p, "\"ae_level\":%d,", s->status.ae_level);
  p += sprintf(p, "\"aec_value\":%u,", s->status.aec_value);
  p += sprintf(p, "\"agc\":%u,", s->status.agc);
  p += sprintf(p, "\"agc_gain\":%u,", s->status.agc_gain);
  p += sprintf(p, "\"gainceiling\":%u,", s->status.gainceiling);
  p += sprintf(p, "\"bpc\":%u,", s->status.bpc);
  p += sprintf(p, "\"wpc\":%u,", s->status.wpc);
  p += sprintf(p, "\"raw_gma\":%u,", s->status.raw_gma);
  p += sprintf(p, "\"lenc\":%u,", s->status.lenc);
  p += sprintf(p, "\"vflip\":%u,", s->status.vflip);
  p += sprintf(p, "\"hmirror\":%u,", s->status.hmirror);
  p += sprintf(p, "\"dcw\":%u,", s->status.dcw);
  p += sprintf(p, "\"colorbar\":%u,", s->status.colorbar);
  // Serial.println("i am here in status handler");
  p += sprintf(p, "\"face_detect\":%u,", detection_enabled);
  p += sprintf(p, "\"face_enroll\":%u,", is_enrolling);
  p += sprintf(p, "\"face_recognize\":%u", recognition_enabled);
  *p++ = '}';
  *p++ = 0;
  httpd_resp_set_type(req, "application/json");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  return httpd_resp_send(req, json_response, strlen(json_response));
}

// the below code is to generate the HTML file in the UI PAGE

static esp_err_t index_handler(httpd_req_t *req) {
  httpd_resp_set_type(req, "text/html");
  httpd_resp_set_hdr(req, "Content-Encoding", "gzip");
  sensor_t *s = esp_camera_sensor_get();
  if (s->id.PID == OV3660_PID) {
    return httpd_resp_send(req, (const char *)index_ov3660_html_gz, index_ov3660_html_gz_len);
  }
  return httpd_resp_send(req, (const char *)index_ov2640_html_gz, index_ov2640_html_gz_len);
}

//the below function is calling when the esp32 is not connecting with any WIFI
//this function will generate one TCP connection with khadas and mobile by generating one hotspot
//through that hotspot mobile will connect to the ESP32 and the user will send the WIFI credentials
//after getting the WIFI credentials we are storing them in internal storage and we able to connect to WIFI here

int server_fd, new_socket, valread;
struct sockaddr_in address;
int opt = 1;
int addrlen = sizeof(address);
char buffer[1024] = { 0 };
// char *hello = "Hello from server";

void startWifiConnection() {

  WiFi.softAP(wifi_ssid); // here generating the hotspot of ESP32 without any password, only ssid
  int server_fd = socket(AF_INET, SOCK_STREAM, 0);
  if (server_fd < 0) {
    Serial.print("socket failed");
    Serial.print(server_fd);
  }
  int setsoc = setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &opt, sizeof(opt));
  if (setsoc) {
    Serial.print("setsockopt");
    Serial.print(setsoc);
  }
  address.sin_family = AF_INET;
  address.sin_addr.s_addr = INADDR_ANY;
  address.sin_port = htons(PORT);
  int bindData = bind(server_fd, (struct sockaddr *)&address, sizeof(address)); // here we are connecting to the socket 
  if (bindData < 0) {
    Serial.print("bind failed");
    Serial.print(bindData);
  }
  if (listen(server_fd, 5) < 0) {
    Serial.print("listen");
  }
  while (true) {
    new_socket = accept(server_fd, (struct sockaddr *)&address, (socklen_t *)&addrlen);
    if (new_socket < 0) {
      Serial.print("accept failed..");
    } else {
      Serial.print("accept success");
    }
    Serial.print("starting: " + new_socket);
    char buffer1[1024] = { 0 };
    valread = read(new_socket, buffer1, 1024);
    Serial.println("");
    Serial.print(buffer1);
    //send(new_socket, "id", 2, 0);
    //sleep(2);
    //char buffer1[1024] = { 0 };
    //valread = read(new_socket, buffer1, 1024);
    Serial.println("");
    if (buffer1[0] == 's' && buffer1[1] == 's' && buffer1[2] == 'i' && buffer1[3] == 'd') {
      String val(buffer1);
      int v = val.indexOf("\r\n");
      const String ssid = val.substring(6, v);
      val = val.substring(v+2,val.length());
      v = val.indexOf("\r\n");
      const String password = val.substring(10, v);
      val = val.substring(v+2,val.length());
      const String system_ip = val.substring(11, val.length()-2);

      Serial.println(ssid);
      Serial.println("ssid done");
      Serial.println(password);
      Serial.println("password done");
      Serial.println(system_ip);
      Serial.println("ip done");
      WiFi.begin(ssid.c_str(), password.c_str());
      //WiFi.begin("Jio FBOX 4G","FBox@505");
      while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.println("failed to connect");
      }
      Serial.println("");
      Serial.println("WiFi connected");
      Serial.print("Camera Ready! Use 'http://");
      Serial.print(WiFi.localIP());
      Serial.println("' to connect");
      close(new_socket);
      shutdown(server_fd, SHUT_RDWR);
      // store in the disk...
      preferences.begin("my-app", false);
      preferences.putString("present", "yes");
      preferences.putString("ssid", ssid);
      preferences.putString("password", password);
      preferences.putString("system_ip", system_ip);
      preferences.end();
      //sending_msg = "FBoxAIData:"+String(WiFi.localIP());
      //Clientcreation(sending_msg);
      break;
    }
  }
  startCameraServer();
}


// this is the only function which starting the process

void startCameraServer() {
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();

  httpd_uri_t index_uri = {
    .uri = "/",
    .method = HTTP_GET,
    .handler = index_handler,
    .user_ctx = NULL
  };

  httpd_uri_t status_uri = {
    .uri = "/status",
    .method = HTTP_GET,
    .handler = status_handler,
    .user_ctx = NULL
  };

  httpd_uri_t cmd_uri = {
    .uri = "/control",
    .method = HTTP_GET,
    .handler = cmd_handler,
    .user_ctx = NULL
  };

  httpd_uri_t capture_uri = {
    .uri = "/capture",
    .method = HTTP_GET,
    .handler = capture_handler,
    .user_ctx = NULL
  };

  httpd_uri_t stream_uri = {
    .uri = "/stream",
    .method = HTTP_GET,
    .handler = stream_handler,
    .user_ctx = NULL
  };


  ra_filter_init(&ra_filter, 20);

  mtmn_config.type = FAST;
  mtmn_config.min_face = 80;
  mtmn_config.pyramid = 0.707;
  mtmn_config.pyramid_times = 4;
  mtmn_config.p_threshold.score = 0.6;
  mtmn_config.p_threshold.nms = 0.7;
  mtmn_config.p_threshold.candidate_number = 20;
  mtmn_config.r_threshold.score = 0.7;
  mtmn_config.r_threshold.nms = 0.7;
  mtmn_config.r_threshold.candidate_number = 10;
  mtmn_config.o_threshold.score = 0.7;
  mtmn_config.o_threshold.nms = 0.7;
  mtmn_config.o_threshold.candidate_number = 1;
  

  face_id_init(&id_list, FACE_ID_SAVE_NUMBER, ENROLL_CONFIRM_TIMES); // this is the initialisation of the &id_list, FACE_ID_SAVE_NUMBER, ENROLL_CONFIRM_TIMES
  
  SD_ENROLL(); // here i am calling the SD_ENROLL function to enroll all ID's which are present in SD card

  Serial.printf("Starting web server on port: '%d'\n", config.server_port);
  if (httpd_start(&camera_httpd, &config) == ESP_OK) {
    httpd_register_uri_handler(camera_httpd, &index_uri);
    httpd_register_uri_handler(camera_httpd, &cmd_uri);
    httpd_register_uri_handler(camera_httpd, &status_uri);
    httpd_register_uri_handler(camera_httpd, &capture_uri);
  }

  config.server_port += 1;
  config.ctrl_port += 1;
  Serial.printf("Starting stream server on port: '%d'\n", config.server_port);
  if (httpd_start(&stream_httpd, &config) == ESP_OK) {
    httpd_register_uri_handler(stream_httpd, &stream_uri);
  }
}
// the below function is to connect with khadas and send some message 
// by using server socket connection we are doing this

WiFiClient client;
int connected = 0;
void Clientcreation(String sending_msg) {
  if(connected == 0) {
    const uint16_t port = 7240;

    // find with the prefrence....
    
    preferences.begin("my-app", false);
    String present = "192.168.0.139";//preferences.getString("system_ip","no");
    preferences.end();
    const char *host = present.c_str();//"192.168.29.77";// khadas ip which it sends address to
    Serial.println("first time ");
    Serial.println(present);
    if (!client.connect(host, port)) { 
      Serial.println("Connection to host failed");
      delay(1000);
      return;
    } 
    connected = 1;
    return;
  } else {
    Serial.println("next time ");
    connected = 1;
  }
  client.print(sending_msg);
  Serial.println("sending done");
}