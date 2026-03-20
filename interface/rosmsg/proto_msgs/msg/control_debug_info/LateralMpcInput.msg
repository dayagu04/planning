  # parameters
float64 q_y 
float64 q_phi 
float64 q_omega 
float64 q_phi_WN 
float64 q_y_WN 
float64 omega_limit 
float64 delta_limit 

  # init state
LateralMpcState init_state 
  # reference
float64[] curv_factor_vec 
float64[] dx_ref_mpc_vec 
float64[] dy_ref_mpc_vec 
float64[] dphi_ref_mpc_vec 
float64[] vel_lat_ctrl_mpc_vec 