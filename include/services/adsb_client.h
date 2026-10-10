#pragma once

#include <cstddef>
#include <cstdint>

namespace services::adsb {

struct Aircraft {
  char id[8];
  float lat;
  float lon;
  float nose_deg;
  float track_deg;
  float gs_knots;
  /** Age of the position fix at fetch time (ms), from the feed's seen_pos.
   *  Added to the elapsed time when dead-reckoning so the drawn position
   *  reflects the estimated current location, not the already-stale fix. */
  uint32_t pos_age_ms;
  char callsign[9];
  char type[5];
  char alt[12];
};

constexpr size_t kMaxAircraft = 64;

size_t aircraftCount();
const Aircraft* aircraftList(size_t* count);

/** Hook invoked during long HTTP I/O (e.g. wifiLoop). Optional. */
using PollFn = void (*)();
void setPollFn(PollFn fn);

/** Fetch aircraft within fetch_radius_km of center_lat/lon from adsb.fi. */
bool fetchUpdate(double center_lat, double center_lon, float fetch_radius_km);

}  // namespace services::adsb
