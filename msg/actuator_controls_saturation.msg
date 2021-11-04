uint64 timestamp  # time since system start (microseconds)

uint16 bitmask    # bitmask of the flags below
bool valid        # 0 - true when the saturation status is used
bool roll_pos     # 1 - true when a positive roll demand change will increase saturation
bool roll_neg     # 2 - true when a negative roll demand change will increase saturation
bool pitch_pos    # 3 - true when a positive pitch demand change will increase saturation
bool pitch_neg    # 4 - true when a negative pitch demand change will increase saturation
bool yaw_pos      # 5 - true when a positive yaw demand change will increase saturation
bool yaw_neg      # 6 - true when a negative yaw demand change will increase saturation
bool thrust_x_pos # 7 - true when a forward thrust demand change will increase saturation
bool thrust_x_neg # 8 - true when a backward thrust demand change will increase saturation
bool thrust_y_pos # 9 - true when a right thrust demand change will increase saturation
bool thrust_y_neg #10 - true when a left thrust demand change will increase saturation
bool thrust_z_pos #11 - true when a downward thrust demand change will increase saturation
bool thrust_z_neg #12 - true when a upward thrust demand change will increase saturation
