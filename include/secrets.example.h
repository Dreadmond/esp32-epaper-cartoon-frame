/* SECRETS TEMPLATE
 * Copy this file to secrets.h and fill in your values.
 * DO NOT commit secrets.h to version control!
 */

#ifndef SECRETS_H
#define SECRETS_H

// =============================================================================
// WIFI CREDENTIALS
// =============================================================================
#define SECRET_WIFI_SSID     "your_wifi_ssid"
#define SECRET_WIFI_PASSWORD "your_wifi_password"

// =============================================================================
// OPENWEATHERMAP API (optional)
// Get your free API key at: https://openweathermap.org/api
// =============================================================================
#define SECRET_OWM_APIKEY    "your_openweathermap_api_key"

// =============================================================================
// LOCATION (optional)
// Find coordinates at: https://www.latlong.net/
// =============================================================================
#define SECRET_LAT           "51.5074"
#define SECRET_LON           "-0.1278"
#define SECRET_CITY_STRING   "London"

// =============================================================================
// HOME ASSISTANT (optional)
// Generate a Long-Lived Access Token in HA: Profile -> Security -> Long-Lived Access Tokens
// =============================================================================
#define SECRET_HA_HOST            "homeassistant.local"
#define SECRET_HA_PORT            8123
#define SECRET_HA_TOKEN           "your_ha_long_lived_token"
#define SECRET_HA_TEMP_ENTITY     "sensor.temperature"
#define SECRET_HA_HUMIDITY_ENTITY "sensor.humidity"

// =============================================================================
// MQTT (for Home Assistant telemetry)
// =============================================================================
#define SECRET_MQTT_BROKER    "homeassistant.local"
#define SECRET_MQTT_PORT      1883
#define SECRET_MQTT_USERNAME  "mqtt_user"
#define SECRET_MQTT_PASSWORD  "mqtt_password"
#define SECRET_MQTT_CLIENT_ID "cartoon_frame"

// =============================================================================
// NEXTCLOUD WEBDAV (for photo/cartoon display modes)
// =============================================================================
#define SECRET_NEXTCLOUD_URL      "https://your-nextcloud.com/remote.php/dav/files/username/path/"
#define SECRET_NEXTCLOUD_USER     "your_nextcloud_user"
#define SECRET_NEXTCLOUD_PASS     "your_nextcloud_password"
#define SECRET_NEXTCLOUD_PHOTO    "photo.png"
#define SECRET_NEXTCLOUD_CARTOON  "cartoon.png"

// =============================================================================
// GITHUB OTA (optional - for over-the-air updates)
// =============================================================================
#define SECRET_GITHUB_OWNER  "your_github_username"
#define SECRET_GITHUB_REPO   "your_repo_name"
#define SECRET_GITHUB_TOKEN  ""  // Leave empty for public repos

#endif // SECRETS_H

