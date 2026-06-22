# Aggregated GNSS quality / satellite status for the WitMotion WTGPS-300.
# Combines data from NMEA GSA (fix type + DOP) and GSV (satellites in view).
std_msgs/Header header

uint8 FIX_NONE = 1
uint8 FIX_2D   = 2
uint8 FIX_3D   = 3
uint8 fix_type              # One of FIX_* above (from GSA)

int32 satellites_visible    # Satellites in view (from GSV)
int32 satellites_used       # Satellites used in the position fix (from GGA)

float32 pdop                # Position (3D) dilution of precision
float32 hdop                # Horizontal dilution of precision
float32 vdop                # Vertical dilution of precision

SatelliteInfo[] satellites  # Per-satellite details (from GSV)
