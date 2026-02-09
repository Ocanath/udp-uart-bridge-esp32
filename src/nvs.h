#ifndef NVS_H
#define NVS_H
#include <Preferences.h>

#define WIFI_MAX_SSID_LEN   32    //SSID max length is 32 characters
#define WIFI_MAX_PWD_LEN    63    //WPA2-PSK key length limit is 63 characters
#define DEVICE_NAME_LEN     32
//add device name property and UDP scannable method to retrieve it, for IP-name mapping
typedef struct nvs_settings_t
{
  char ssid[WIFI_MAX_SSID_LEN];
  char password[WIFI_MAX_PWD_LEN];
  int port; //udp server forwarding port
  int reply_offset;	//offset applied to remote port for splitting send and receieve ports. default 0
  char name[DEVICE_NAME_LEN];
  uint8_t en_fixed_target;
  uint32_t remote_target_ip;
}nvs_settings_t;

extern Preferences preferences;
extern nvs_settings_t gl_prefs;


void init_prefs(Preferences * p, nvs_settings_t * s);

#endif