float32 clearance_score
float32 narrow_score
float32 turn_score
float32 efficiency_score
float32 heuristic

# Raw geometry metrics (un-normalized).
float32 min_clearance_m
float32 narrow_fraction
float32 max_turn_rad
float32 path_length_m
float32 straight_dist_m
float32 detour_ratio

# Monte Carlo local risk profile sampled along the path (0=low risk, 1=high risk).
float32[] local_risks
