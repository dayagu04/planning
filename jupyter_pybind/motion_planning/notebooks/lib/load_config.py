import json
import math
class LoadConfig():
    def __init__(self, path, model):
        self.data = {}
        if(model == "hcp"):
            self.model = "control_parameters_hcp"
        elif(model == "apa"):
            self.model = "control_parameters_apa"
        with open(path, 'r') as f:
            self.json = json.load(f)


    def GenerateData(self):
        config = self.json["strategies"][0]["config"][self.model]

        load_list = ["mpc_horizon", "mpc_fs", "lat_q0", "lat_q1", "lat_r", "state_size", "input_size",
            "lat_dy_limit", "lat_dphi_limit", "lat_steering_angle_limit_deg", "lat_steering_angle_rate_limit_deg",
            "curv_factor_k1", "curv_factor_k2", "lat_mpc_vel_min", "lat_mpc_vel_min_parking", "lat_acc_y_limit", "lat_jerk_y_limit",
            "lat_actuator_delay", "control_fs", "lat_ctrl_index", "curv_factor_gain"]
        try:
            for item in load_list:
                self.data[item] = config[item]
        except:
            print('error')
        # if(self.model == "control_parameters_apa"):
        #     self.data["vel_min"] = self.data["lat_mpc_vel_min_parking"]
        # else:
        #     try:
        #         self.data["vel_min"] = self.data["lat_mpc_vel_min"]
        #     except:
        #         print('error')
        try:
            vehicle_config = self.json["strategies"][0]["config"]["vehicle_param_value"]
            self.data["steer_ratio"] = vehicle_config["steer_ratio_v"][0]
            self.data["max_frontwheel_steer_at_v_kph"] = vehicle_config["max_frontwheel_steer_at_v_kph"]
            self.data["max_frontwheel_at_steer_deg"] = vehicle_config["max_frontwheel_at_steer_deg"]
            self.data["max_frontwheel_at_steer_rate_degps"] = vehicle_config["max_frontwheel_at_steer_rate_degps"]

            self.data["max_deceleration_at_v_mps"] = vehicle_config["max_deceleration_at_v_mps"]
            self.data["max_deceleration_at_acc_mps2"] = vehicle_config["max_deceleration_at_acc_mps2"]
            self.data["max_acceleration_at_v_mps"] = vehicle_config["max_acceleration_at_v_mps"]
            self.data["max_acceleration_at_acc_mps2"] = vehicle_config["max_acceleration_at_acc_mps2"]
            self.data["max_decjerk_at_v_mps"] = vehicle_config["max_decjerk_at_v_mps"]
            self.data["max_decjerk_at_jerk_mps3"] = vehicle_config["max_decjerk_at_jerk_mps3"]
            self.data["max_accjerk_at_v_mps"] = vehicle_config["max_accjerk_at_v_mps"]
            self.data["max_accjerk_at_jerk_mps3"] = vehicle_config["max_accjerk_at_jerk_mps3"]
            self.data["lat_steering_angle_limit"] = math.radians(self.data["lat_steering_angle_limit_deg"])
            self.data["lat_steering_angle_rate_limit"] = math.radians(self.data["lat_steering_angle_rate_limit_deg"])
        except:
            print('error')

        load_list = ["rear_axle_to_head", "rear_axle_to_back", "half_width"]
        try:
            for item in load_list:
                self.data[item] = vehicle_config[item]
        except:
            self.data["rear_axle_to_head"] = 4.015
            self.data["rear_axle_to_back"] = 1.083
            self.data["half_width"] = 0.98

        return self.data

