# Per-frame capture metadata of a LUCID Arena camera (image_raw/metadata).
#
# The fields up to device_temperature_c have the same names and meaning as
# flir_spinnaker_camera/FlirMetadata, so RAW tools can read either type the same way.
# The leading block (header .. camera_timestamp_ns) also has the same layout, which
# DM_clipGUI's bag_diagnostics relies on to count dropped frames.

std_msgs/Header header

uint32 width
uint32 height
uint32 step

string encoding
# PixelFormat as transmitted by the camera (BayerRG8 / BayerRG12p / BayerRG16 / BayerRG24 ...)
string pixel_format

# GVSP block id (+1 per frame; 64-bit with GevGVSPExtendedIDMode On)
uint64 camera_frame_id
# Camera clock at the frame (chunk Timestamp when on), ns
uint64 camera_timestamp_ns

bool acquisition_frame_rate_enable
float64 acquisition_frame_rate_hz

string exposure_auto
float64 exposure_time_us

string gain_auto
float64 gain_db

# Chunk BlackLevel (camera units, percent of the 12-bit range on the TRI054S); see black_level_raw
float64 black_level

string balance_white_auto

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
float64 balance_ratio_red
float64 balance_ratio_blue
bool lut_enable
# Saturation code of this output in image_raw units (parameter raw.white_level_dn, 0 = unknown)
uint32 white_level_dn

# Slowly varying: last DeviceTemperature read (temperature.interval_sec, default 1 s), NaN on failure
float64 device_temperature_c

# ---- LUCID / Sony IMX490 specific ----
# false while the chunk exposure / gain / black level differ from either of the two previous
# frames. On the TRI054S the chunk value switches one frame before the pixels do (measured
# 2026-10-19), so the first frames after a change can carry the old exposure in the image.
bool settled
# CFA of image_raw (IspBayerPattern / pixel format): RGGB, GRBG, GBRG, BGGR, or "" for mono
string cfa_pattern
# Bits per pixel as transmitted (8, 12, 16, 24); image_raw holds them in 8-, 16- or 32-bit words
uint8 bits_per_pixel
# Pedestal in image_raw units (BlackLevelRaw with BlackLevelSelector=All)
int64 black_level_raw
# HDR | HDRChannel0 .. HDRChannel3 (which sub-pixel / conversion gain the image is)
string hdr_output
bool hdr_image_enhancement_enable
bool hdr_digital_clamping_enable
bool hdr_tuning_enable
# Per HDR channel 0..3 (HDRTuningChannelSelector), NaN if unreadable
float64[4] hdr_channel_analog_gain_db
float64[4] hdr_channel_exposure_time_us
bool balance_white_enable
float64 balance_ratio_green
bool color_transformation_enable
string lut_tone_mapping
bool reverse_x
bool reverse_y
string sensor_shutter_mode

# Slowly varying: last DeviceUpTime read with the temperature (seconds since the camera powered up), NaN on
# failure. A smaller value than in an earlier frame means the camera rebooted (settings and ForceIP are lost).
float64 device_uptime_s
