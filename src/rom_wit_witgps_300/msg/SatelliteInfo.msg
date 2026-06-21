# Information about a single satellite, parsed from NMEA GSV sentences.
int32 prn        # Satellite PRN / ID number
int32 elevation  # Elevation in degrees (0..90)
int32 azimuth    # Azimuth in degrees (0..359)
int32 snr        # Signal-to-noise ratio in dB-Hz (-1 if not tracking)
