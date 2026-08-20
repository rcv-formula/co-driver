uint8 MOTION_UNKNOWN=0
uint8 STATIC=1
uint8 DYNAMIC=2
uint8 WALL_STATIC=3

std_msgs/Header header

float32 center_x
float32 center_y

float32 min_x
float32 min_y
float32 max_x
float32 max_y

float32 min_range
float32 max_range

float32 width
uint32 point_count
uint32 scan_start_index
uint32 scan_end_index
float32 scan_start_angle
float32 scan_end_angle

uint32 track_id
uint8 motion_label
bool is_wall_static
bool is_density_static
float32 max_dense_weight
uint32 observed_count
uint32 static_hit_count
uint32 dynamic_hit_count
bool ego_motion_gate_active

float32 velocity_x
float32 velocity_y
float32 speed
float32 compensated_displacement

float32 risk_weight
