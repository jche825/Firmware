int8 LANDING_GEAR_UP = 1	# landing gear up
int8 LANDING_GEAR_DOWN = -1	# landing gear down

float32 roll_body				# body angle in NED frame
float32 pitch_body				# body angle in NED frame
float32 yaw_body				# body angle in NED frame

float32 yaw_sp_move_rate		# rad/s (commanded by user)

# For quaternion-based attitude control
float32[4] q_d					# Desired quaternion for quaternion control
bool q_d_valid					# Set to true if quaternion vector is valid

float32 thrust					# Output thrust in Newton the power system should generate
float32[3] thrust_vec_1         # Attitude thrust from baseline controller
float32[3] thrust_vec_2         # Attitude thrust from secondary controller
float32[2] hor_thrust           # Output x/y horizontal thrust commands
float32[2] hor_thrust_2         # x/y horizontal thrust from secondary controller 
int8 controller_selected        # Controller selected (0: baseline, 1: secondary)
float32[3] pos_int              # position integral

bool roll_reset_integral			# Reset roll integral part (navigation logic change)
bool pitch_reset_integral			# Reset pitch integral part (navigation logic change)
bool yaw_reset_integral				# Reset yaw integral part (navigation logic change)

bool fw_control_yaw					# control heading with rudder (used for auto takeoff on runway)
bool disable_mc_yaw_control			# control yaw for mc (used for vtol weather-vane mode)

bool apply_flaps

float32 landing_gear

# Debug
uint32 loop_counter
float32 dt

# TOPICS vehicle_attitude_setpoint mc_virtual_attitude_setpoint fw_virtual_attitude_setpoint
