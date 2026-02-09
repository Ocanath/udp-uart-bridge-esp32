#include "nvs.h"

Preferences preferences;
nvs_settings_t gl_prefs = {0};

void init_prefs(Preferences * p, nvs_settings_t * s)
{
  p->begin("prefs", false); 
  p->getBytes("settings", s, sizeof(nvs_settings_t));
}
