class Modc:

    def __init__(self):
        self.num_sensors = 8
        #self.sensor_dist = sensor_dist  # will be provided in np.array form
        #self.detection_obj = det_obj  # will be provided in np.array form
        self.danger_nr = float()  # Danger associated with no of robots connected with robot
        self.count = 0

    def no_neigh_rob(self, target_no_robots, diffusion_rate, det_obj):  # An external danger # Decided diffusion rate is 0.025
        #self.count = self.count + 1
        #no_nei_rob = (det_obj == 1).sum()  # Count number of robots connected with robot
        no_nei_rob = len(det_obj)
        if no_nei_rob >= target_no_robots:  # Diffuse danger when current no robots == target no of robots
            #self.danger_nr = (1 - diffusion_rate) * self.danger_nr
            self.danger_nr = 0.0
        else:
            self.danger_nr = self.danger_nr + ((target_no_robots - no_nei_rob) * 0.02)
        self.danger_nr = 0 if self.danger_nr > 1 else self.danger_nr
        danger_status = Modc.value_dc(self.danger_nr)
        return danger_status

    def pamp_vision(self, signal_value, limit):
        if signal_value >= limit:
            self.pamp_value = 0.1
        else:
            self.pamp_value = 0.0
            pass

    def commulative_danger_signal(self, wd, wp):
        pass

    # Base on danger theory immune cell must respond to harmful signal not necessarily to foreign bodies
    # Both safe and internal danger signal will be generated from it.

    def inflammation(self):
        pass
    '''def self_nonself(self, rob_min_limit, rob_max_limit, obstacle_limit):
        for i in range(self.num_sensors):
            if self.detection_obj[i]:  # Presence of robot
                if not (rob_min_limit < self.sensor_dist[i] > rob_max_limit):  # danger produce by robot
                    self.ext_danger[i] = self.ext_danger[i] + 0.04
                    self.safe_signal[i] = self.safe_signal[i] - 0.04
                else:  # robot in safe range
                    self.ext_danger[i] = self.ext_danger[i] - 0.04
                    self.safe_signal[i] = self.safe_signal[i] + 0.04
            elif not self.detection_obj[i]:  # Presence of Obstacle
                if self.sensor_dist[i] < obstacle_limit :  # danger produce by obstacle
                    self.ext_danger[i] = self.ext_danger[i] + 0.04
                    self.safe_signal[i] = self.safe_signal[i] - 0.04
                else:  # obstacle in safe range
                    self.ext_danger[i] = self.ext_danger[i] - 0.04
                    self.safe_signal[i] = self.safe_signal[i] + 0.04
            else:  # nothing in front of sensor
                continue'''

    @staticmethod
    def value_dc(x):
        if 0 <= x < 0.3:
            return int(1)  # Immature
        elif 0.3 <= x < 0.7:
            return int(2)  # Semi-mature
        elif 0.7 <= x < 1.0:
            return int(3)  # Fully mature
        else:
            return int(0)  # Beginning of New DC

