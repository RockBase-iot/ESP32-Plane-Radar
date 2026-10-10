/**
 * Plane Radar — WiFi setup, then radar UI on the round GC9A01 display.
 */

#include <Arduino.h>
#include <WiFi.h>

#include <time.h>

#include "config.h"
#include "hardware/display.h"
#include "services/adsb_client.h"
#include "services/radar_location.h"
#include "services/time_settings.h"
#include "services/wifi_setup.h"
#include "ui/radar_animation_policy.h"
#include "ui/radar_display.h"
#include "ui/radar_range.h"
#include "ui/status_screens.h"
#ifdef BOARD_NM_TV_154
#include "ui/nm_tv_154_policy.h"
#endif

namespace {

volatile bool g_radar_visible = false;
unsigned long g_wifi_down_since = 0;
unsigned long g_last_reconnect_ms = 0;
unsigned long g_last_adsb_fetch_ms = 0;
unsigned long g_last_animation_refresh_ms = 0;
portMUX_TYPE g_adsb_update_lock = portMUX_INITIALIZER_UNLOCKED;
TaskHandle_t g_adsb_fetch_task = nullptr;
volatile bool g_adsb_refresh_pending = false;
volatile unsigned long g_background_adsb_update_ms = 0;
struct AdsbRequest {
  bool enabled;
  double lat;
  double lon;
  float radius_km;
};
AdsbRequest g_adsb_request = {};
#ifdef BOARD_NM_TV_154
constexpr uint16_t kTouchPressedThreshold = 90;
constexpr unsigned long kSquareStatusRefreshIntervalMs = 1000UL;
unsigned long g_last_square_status_refresh_ms = 0;
bool g_clock_started = false;
ui::nm_tv_154::TouchRangeState g_touch_range_state;
#endif

#ifdef BOARD_NM_TV_154
constexpr bool kIsNmTv154 = true;
#else
constexpr bool kIsNmTv154 = false;
#endif

void startLocalClock() {
#ifdef BOARD_NM_TV_154
  if (g_clock_started || WiFi.status() != WL_CONNECTED) {
    return;
  }
  configTime(0, 0, config::kNtpServer);
  g_clock_started = true;
  Serial.printf("Clock sync: %s\n", config::kNtpServer);
#endif
}

void showRadarIfConnected() {
  if (WiFi.status() != WL_CONNECTED) {
    g_radar_visible = false;
    return;
  }
  startLocalClock();
  const unsigned long draw_started_ms = millis();
  ui::radarDisplayDraw();
  g_last_animation_refresh_ms = draw_started_ms;
  g_radar_visible = true;
}

void onRangeTap() {
  ui::radar::rangeNext();
  char range_label[12];
  ui::radar::formatCurrentRing3Label(range_label, sizeof(range_label));
  Serial.printf("Range: %s (outer ~%.0f km)\n", range_label,
                ui::radar::rangeCurrent().outer_km);

  if (g_radar_visible && WiFi.status() == WL_CONNECTED) {
    const unsigned long draw_started_ms = millis();
    ui::radarDisplayDraw();
    g_last_animation_refresh_ms = draw_started_ms;
  }
}

void handleNmTv154RangeTouch() {
#ifdef BOARD_NM_TV_154
  const uint16_t raw = touchRead(T9);
  const bool is_down = raw < kTouchPressedThreshold;
  const bool was_down = g_touch_range_state.was_down;
  g_touch_range_state = ui::nm_tv_154::nextTouchRangeState(g_touch_range_state,
                                                            is_down);
  if (is_down && !was_down) {
    Serial.printf("[touch] pressed, raw=%u\n", static_cast<unsigned>(raw));
  }
  if (g_touch_range_state.range_tap) {
    Serial.println("[touch] tap -> range");
    onRangeTap();
  }
#endif
}

void serviceRadarAnimation() {
  if (!g_radar_visible || WiFi.status() != WL_CONNECTED) {
    return;
  }

  const unsigned long now_ms = millis();
  if (!ui::radar::animationNeeded(ui::radar::showSweep(),
                                  services::adsb::aircraftCount()) ||
      now_ms - g_last_animation_refresh_ms <
          ui::radar::animationIntervalMs(kIsNmTv154)) {
    return;
  }

  g_last_animation_refresh_ms = now_ms;
  ui::radarDisplayRefreshAircraft();
}

void pollNetwork() {
  wifiLoop();
  serviceRadarAnimation();
  handleNmTv154RangeTouch();
}

void pollBackgroundNetwork() { taskYIELD(); }

void updateAdsbRequest() {
  const AdsbRequest request = {
      g_radar_visible && WiFi.status() == WL_CONNECTED,
      services::location::lat(), services::location::lon(),
      ui::radar::fetchRadiusKm()};
  taskENTER_CRITICAL(&g_adsb_update_lock);
  g_adsb_request = request;
  taskEXIT_CRITICAL(&g_adsb_update_lock);
}

void fetchAndRequestAircraft() {
  taskENTER_CRITICAL(&g_adsb_update_lock);
  const AdsbRequest request = g_adsb_request;
  taskEXIT_CRITICAL(&g_adsb_update_lock);
  if (!request.enabled || WiFi.status() != WL_CONNECTED ||
      !services::adsb::fetchUpdate(request.lat, request.lon, request.radius_km)) {
    return;
  }

  taskENTER_CRITICAL(&g_adsb_update_lock);
  g_background_adsb_update_ms = millis();
  g_adsb_refresh_pending = true;
  taskEXIT_CRITICAL(&g_adsb_update_lock);
}

bool consumeBackgroundAdsbUpdate(unsigned long* update_ms) {
  taskENTER_CRITICAL(&g_adsb_update_lock);
  const bool pending = g_adsb_refresh_pending;
  if (pending) {
    *update_ms = g_background_adsb_update_ms;
    g_adsb_refresh_pending = false;
  }
  taskEXIT_CRITICAL(&g_adsb_update_lock);
  return pending;
}

void adsbFetchTask(void*) {
  for (;;) {
    fetchAndRequestAircraft();
    vTaskDelay(pdMS_TO_TICKS(config::kAdsbFetchIntervalMs));
  }
}

void handleBootButton() {
  bootButtonPollLongPress();
  if (bootButtonConsumeTap()) {
    onRangeTap();
  }
  handleNmTv154RangeTouch();
}

void fetchAndDrawAircraft() {
  const float fetch_km = ui::radar::fetchRadiusKm();
  if (!services::adsb::fetchUpdate(services::location::lat(),
                                   services::location::lon(), fetch_km)) {
#ifdef BOARD_NM_TV_154
    ui::radarDisplayRefreshAircraft();
#endif
    handleBootButton();
    return;
  }
  const unsigned long draw_started_ms = millis();
  ui::radarDisplayMarkDataUpdated(millis());
  ui::radarDisplayRefreshAircraft();
  g_last_animation_refresh_ms = draw_started_ms;
  handleBootButton();
}

}  // namespace

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println();
  Serial.printf("Plane Radar %s\n", config::kFirmwareVersion);

  bootButtonInit();
  displayInit();
  if (wifiShowsSetupScreenOnBoot()) {
    statusScreenPortal();
  }
  services::location::init();
  services::time_settings::init();
  ui::radar::rangeInit();
  services::adsb::setPollFn(pollBackgroundNetwork);
#ifdef BOARD_NM_TV_154
  Serial.printf("[touch] ready, raw=%u\n",
                static_cast<unsigned>(touchRead(T9)));
#endif

  if (wifiSetupConnect()) {
    showRadarIfConnected();
  }
  updateAdsbRequest();
  if (xTaskCreatePinnedToCore(adsbFetchTask, "adsb-fetch", 16384, nullptr, 1,
                              &g_adsb_fetch_task, 0) != pdPASS) {
    Serial.println("adsb: background task create failed");
    services::adsb::setPollFn(pollNetwork);
  }
}

void loop() {
  handleBootButton();
  wifiLoop();

  if (WiFi.status() != WL_CONNECTED) {
    if (g_radar_visible) {
#ifdef BOARD_NM_TV_154
      ui::radarDisplayRefreshAircraft();
#endif
      Serial.println("WiFi lost — will reconnect");
      g_radar_visible = false;
    }

    if (g_wifi_down_since == 0) {
      g_wifi_down_since = millis();
    }

    const unsigned long down_ms = millis() - g_wifi_down_since;
    if (down_ms >= config::kWifiDownGraceMs &&
        millis() - g_last_reconnect_ms >= config::kWifiReconnectIntervalMs) {
      g_last_reconnect_ms = millis();
      if (wifiReconnect()) {
        g_wifi_down_since = 0;
        showRadarIfConnected();
      }
    }
  } else {
    g_wifi_down_since = 0;
    if (!g_radar_visible) {
      showRadarIfConnected();
    } else {
      const unsigned long now_ms = millis();
      unsigned long update_ms = 0;
      if (g_adsb_fetch_task == nullptr &&
          now_ms - g_last_adsb_fetch_ms >= config::kAdsbFetchIntervalMs) {
        g_last_adsb_fetch_ms = now_ms;
        fetchAndDrawAircraft();
      } else if (consumeBackgroundAdsbUpdate(&update_ms)) {
        const unsigned long draw_started_ms = millis();
        ui::radarDisplayMarkDataUpdated(update_ms);
        ui::radarDisplayRefreshAircraft();
        g_last_animation_refresh_ms = draw_started_ms;
      } else {
        serviceRadarAnimation();
#ifdef BOARD_NM_TV_154
        const unsigned long status_now_ms = millis();
        if (status_now_ms - g_last_square_status_refresh_ms >=
            kSquareStatusRefreshIntervalMs) {
          g_last_square_status_refresh_ms = status_now_ms;
          ui::radarDisplayRefreshStatus();
        }
#endif
      }
    }
  }

  updateAdsbRequest();
  delay(10);
}
