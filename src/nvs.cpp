#include "nvs.h"
#include <nvs_flash.h>

Preferences preferences;
nvs_settings_t gl_prefs = {0};

void init_prefs(Preferences * p, nvs_settings_t * s)
{
  // Erase and reinitialize NVS if the partition is corrupted or has no free pages.
  // This handles the "invalid header" error that occurs after a manual flash erase.
  esp_err_t err = nvs_flash_init();
  if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND)
  {
    nvs_flash_erase();
    nvs_flash_init();
  }
  p->begin("prefs", false);
  p->getBytes("settings", s, sizeof(nvs_settings_t));
}
