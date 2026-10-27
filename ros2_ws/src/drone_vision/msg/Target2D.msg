# Optical frame: x right, y down, z forward.
std_msgs/Header header
bool valid
int32 tag_id
uint32 image_width
uint32 image_height
float64 center_u
float64 center_v
# Detector corner order, four u,v pixel pairs; not XYZ.
float64[8] corners_uv
float64 area_px2
string reason
bool calibrated
bool pose_valid
float64 tag_size_m
float64[3] translation_m
float64 pose_rms_px
