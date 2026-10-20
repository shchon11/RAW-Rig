std_msgs/Header header

uint32 width
uint32 height
uint32 step

string encoding
string pixel_format

uint64 camera_frame_id
uint64 camera_timestamp_ns

bool acquisition_frame_rate_enable
float64 acquisition_frame_rate_hz

string exposure_auto
float64 exposure_time_us

string gain_auto
float64 gain_db

float64 black_level

bool gamma_enable
float64 gamma

string balance_white_auto

# ---- RAW research capture (docs/raw_capture_noise_calibration.md section 4.3) ----
# Per frame
# true when exposure_time_us, gain_db and black_level all come from this frame's chunk data.
# A frame whose chunk could not be read while chunk data is on has false here and NaN in those
# fields; they are never back-filled from node values.
bool chunk_valid
# true when the CRC chunk is on and this frame's CRC matches its payload
bool crc_ok

# Per session: read once after setup and copied into every message (re-read after a live
# parameter change)
string serial
string model
string firmware_version
string adc_bit_depth
bool black_level_clamping_enable
float64 balance_ratio_red
float64 balance_ratio_blue
bool lut_enable
bool defect_correct_static_enable
bool isp_enable
# Saturation code measured in R1 (parameter raw.white_level_dn, 0 = unknown)
uint32 white_level_dn

# Slowly varying: last DeviceTemperature read (temperature.interval_sec, default 1 s), NaN on failure
float64 device_temperature_c

# Slowly varying: last DeviceUptime read with the temperature (seconds since the camera powered up), NaN on
# failure. A smaller value than in an earlier frame means the camera rebooted (settings and ForceIP are lost).
float64 device_uptime_s
