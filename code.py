

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import math
from scipy.interpolate import interp1d
from tqdm import tqdm 
import time




class SensorDataLoader:
    def __init__(self, paths):
        self.paths = paths

    def load(self):
        self.compass_data = pd.read_json(self.paths['compass'])
        self.odom_data = pd.read_json(self.paths['odom'])
        self.gnss_data = pd.read_json(self.paths['gnss'])
        self.energy_data = pd.read_json(self.paths['energy'])
        self.energy_map = np.load(self.paths['map'])
        return self

class OdometryProcessor:
    def rotate_odom (compass_data, odom_data):
        
        #  ensure compass headings are continuous and consistent for further math
        compass_heading_wrapped = (compass_data['heading'] + np.pi) % (2 * np.pi) - np.pi
        odom_heading = []
        
        # loop is so math.atan2 can be used (not numpy  functions)
        for i in range(0, len(odom_data)):
            dx = odom_data["dx"][i]
            dy = odom_data["dy"][i]
            heading = math.atan2(dy, dx)
            odom_heading.append(heading)
        
        L = min(len(odom_heading), len(compass_heading_wrapped))
        odom_heading = np.array(odom_heading[:L])    
        compass_heading = np.array(compass_heading_wrapped[:L])
        rot_angle =  np.mean(compass_heading_wrapped[1:30]) - np.mean(odom_heading[0:30])
 
        
        # rotation matrix
        cos_a = np.cos(rot_angle)
        sin_a = np.sin(rot_angle)
    
        # rotate odom to compass frame
        dx_rot = odom_data['dx'] * cos_a - odom_data['dy'] * sin_a
        dy_rot = odom_data['dx'] * sin_a + odom_data['dy'] * cos_a
        
        
        return compass_heading,dx_rot, dy_rot




    def fuse_odom_compass(sigma_heading, sigma_odom,compass_heading,dx_rot, dy_rot):
    
    
        # odom position
        odom_x = np.cumsum(dx_rot)
        odom_y = np.cumsum(dy_rot)
    
        # odom distance
        delta_x = np.diff(odom_x)
        delta_y = np.diff(odom_y)
        distance = np.sqrt(delta_x**2 + delta_y**2)
    
        
        sigma_dist = 0.02 * distance  
    
        # propogation - tusting the compass more
        sigma_dx = np.sqrt((sigma_heading * 0.5 * np.cos(compass_heading[:-1]))**2 +  # reduced heading sigma
                           (distance * sigma_dist * np.sin(compass_heading[:-1]))**2)
        sigma_dy = np.sqrt((sigma_heading * 0.5 * np.sin(compass_heading[:-1]))**2 +
                           (distance * sigma_dist * np.cos(compass_heading[:-1]))**2)
    
        # Weights — dominating the compass over the odom
        weights_dx = 1 / (sigma_dx**2 + 1e-6)
        weights_dy = 1 / (sigma_dy**2 + 1e-6)
    
        
        dx_heading = distance * np.cos(compass_heading[:-1])
        dy_heading = distance * np.sin(compass_heading[:-1])
    
        
        dx_fused_filtered = np.convolve(dx_heading * weights_dx, np.ones(5)/5, mode='same') / \
                            np.convolve(weights_dx, np.ones(5)/5, mode='same')
        dy_fused_filtered = np.convolve(dy_heading * weights_dy, np.ones(5)/5, mode='same') / \
                            np.convolve(weights_dy, np.ones(5)/5, mode='same')
    
        # fused trajectory
        x_fused = np.cumsum(dx_fused_filtered)
        y_fused = np.cumsum(dy_fused_filtered)

        return x_fused, y_fused

    
class EnergyMapMatcher:
    def __init__(self, energy_map, energy_data):
        self.energy_map = energy_map.astype(np.float32, copy=False)
        self.energy_data = energy_data



    def push_into_map(self, x_fused, y_fused):
        # first step: pushing trajectory to map bounds. The first time that there is a match with the energy map - stop
            x_shifts = range(0, 1000, 5)
            y_shifts = range(0, 1000, 5)
            for dx_shift in x_shifts:
                for dy_shift in y_shifts:
                    x_shifted = x_fused + dx_shift
                    y_shifted = y_fused + dy_shift
                    valid = ((x_shifted >= 0) & (x_shifted < self.energy_map.shape[1]) &
                             (y_shifted >= 0) & (y_shifted < self.energy_map.shape[0]))
                    if np.all(valid):
                        return x_shifted, y_shifted
            return x_fused, y_fused
    


    def energy_map_matching(self, info):
        # matching the energy of the trajcrtoy to the map by shifting around the map (or parts of it)
        # until the best result is found
        
        x_shifted = info["x_shifted_window"].astype(np.int32, copy=False)
        y_shifted = info["y_shifted_window"].astype(np.int32, copy=False)
        true_energy = info["true_energy_window"].astype(np.float32, copy=False)

        H, W = self.energy_map.shape
        dx_list = np.array(list(info["x_shift_range"]), dtype=np.int32)
        dy_list = np.array(list(info["y_shift_range"]), dtype=np.int32)

        DX, DY = np.meshgrid(dx_list, dy_list, indexing="ij")  
        pairs = np.stack([DX.ravel(), DY.ravel()], axis=1)      
        K = pairs.shape[0]
        N = x_shifted.shape[0]
        min_valid = max(10, int(0.5 * N))

        best_score = np.inf
        best_pair = (0, 0)
        batch_size = 8192
        for start in range(0, K, batch_size):
                end = min(start + batch_size, K)
                dx_batch = pairs[start:end, 0]  
                dy_batch = pairs[start:end, 1]  
    
                X = x_shifted[:, None] + dx_batch[None, :]
                Y = y_shifted[:, None] + dy_batch[None, :]
    
                # mask per sample/shift
                valid = (X >= 0) & (X < W) & (Y >= 0) & (Y < H)
    
                # clamping
                Xc = np.clip(X, 0, W - 1)
                Yc = np.clip(Y, 0, H - 1)
    
                
                Em = self.energy_map[Yc, Xc]
    
                # squared error where valid
                diff = Em - np.asarray(true_energy)[:, None]
                diff_sq = diff * diff
                diff_sq *= valid  # zero out invalid
    
                # RMSE per shift 
                valid_counts = valid.sum(axis=0)  # (B,)
                with np.errstate(divide='ignore', invalid='ignore'):
                    mse = diff_sq.sum(axis=0) / np.maximum(valid_counts, 1)
                    rmse = np.sqrt(mse, dtype=np.float32)
    
                # minimum valid coverage
                rmse[valid_counts < min_valid] = np.inf
    
                # keep track of global best
                bmin = np.argmin(rmse)
                if rmse[bmin] < best_score:
                    best_score = float(rmse[bmin])
                    best_pair = (int(dx_batch[bmin]), int(dy_batch[bmin]))
    
                
    
        # apply  best shift to get alignment
        x_aligned = x_shifted + best_pair[0]
        y_aligned = y_shifted + best_pair[1]

        return x_aligned, y_aligned, best_pair




    def hierarchical_map_matching(self, x_shifted, y_shifted, true_energy, sigma_energy, odom_drift_ratio, levels, best_shift):
        
        # apply energy matching in levels
            for level in levels:
                stride = level['stride']
                radius = level['radius']
                window = level['window']
                x_range = range(best_shift[0] - radius, best_shift[0] + radius, stride)
                y_range = range(best_shift[1] - radius, best_shift[1] + radius, stride)
    
                info = {
                    "x_shift_range": x_range,
                    "y_shift_range": y_range,
                    "x_shifted_window": x_shifted[:window],
                    "y_shifted_window": y_shifted[:window],
                    "true_energy_window": true_energy[:window]
                }
    
                x_aligned, y_aligned, best_shift = self.energy_map_matching(info)
    
                dx = np.diff(x_aligned)
                dy = np.diff(y_aligned)
                distance = np.sum(np.sqrt(dx**2 + dy**2))
                drift_penalty = (distance * odom_drift_ratio) ** 2
                alpha = drift_penalty / (drift_penalty + sigma_energy**2)
                alpha = np.clip(alpha, 0.01, 1.0)
    
                best_shift_np = np.array(best_shift)
                best_shift = (alpha * best_shift_np).astype(int)
    
            return x_aligned, y_aligned, tuple(best_shift)


class TrajectoryStitcher:
    def __init__(self, matcher, levels, sigma_energy, sigma_odom):
        self.matcher = matcher
        self.levels = levels
        self.sigma_energy = sigma_energy
        self.sigma_odom = sigma_odom

    def create_windows(self, length, window_size, step_size):
        
        # create windows for window map matching
        
        windows = [(i, i + window_size) for i in range(0, length - window_size + 1, step_size)]
        if windows[-1][1] < length:
            windows.append((length - window_size, length))
        return windows
    
    

    def match_per_window(self, x_aligned, y_aligned, true_energy, gnss_data, odom_data, window_size=1000, step_size=20):
        
        windows = self.create_windows(len(x_aligned), window_size, step_size)
        
        stitched_x_all = np.zeros_like(x_aligned, dtype=np.float32)
        stitched_y_all = np.zeros_like(y_aligned, dtype=np.float32)
        weight_array    = np.zeros_like(x_aligned, dtype=np.int32)
    
        best_shift = (0, 0)
        prev_best_shift = (0, 0)
        outlier_count = 0
        all_best_shifts = [] 
        
        # begin loop of matching per window
        
        for counter, (start, end) in tqdm(enumerate(windows), total=len(windows), desc="Stitching trajectory windows"):
            x_window = x_aligned[start:end].copy()
            y_window = y_aligned[start:end].copy()
            energy_window = true_energy[start:end]
    
            # re-use hierarchical map matching function with the finest level only
            level = [{**self.levels[2], "window": end - start}]
            x_aligned_win, y_aligned_win, best_shift = self.matcher.hierarchical_map_matching(
                x_window, y_window, energy_window,
                self.sigma_energy, self.sigma_odom,
                level, best_shift
            )
            
            # dealing with outliers
            if abs(prev_best_shift[0] - best_shift[0]) >= 7 or abs(prev_best_shift[1] - best_shift[1]) >= 7:
                best_shift = ((prev_best_shift[0] + best_shift[0]) // 2,
                              (prev_best_shift[1] + best_shift[1]) // 2)
                outlier_count +=1
            prev_best_shift = best_shift
            all_best_shifts.append(best_shift)
    
            stitched_x_all[start:end] += x_aligned_win.astype(np.float32, copy=False)
            stitched_y_all[start:end] += y_aligned_win.astype(np.float32, copy=False)
            weight_array[start:end]   += 1
            
        mask = weight_array > 0
            
        stitched_x_all[mask] = stitched_x_all[mask] / weight_array[mask].astype(np.float32)
        stitched_y_all[mask] = stitched_y_all[mask] / weight_array[mask].astype(np.float32)
        
        # for evaluations
        outlier_percent = (outlier_count/counter)*100

        return stitched_x_all, stitched_y_all, all_best_shifts, outlier_percent

    
class TrajectoryEvaluator:
    def __init__(self, energy_map):
        self.energy_map = energy_map

    def metrics_and_plots(self, stitched_x, stitched_y, gnss_data, odom_data,all_best_shifts, outlier_percent):
        gnss_time = gnss_data["timestamp"]
        traj_time = np.array(odom_data['timestamp'][:-1])
        interp_x = interp1d(gnss_time, gnss_data["x"], bounds_error=False, fill_value="extrapolate")
        interp_y = interp1d(gnss_time, gnss_data["y"], bounds_error=False, fill_value="extrapolate")


        
        ### Accuracy Metrics ###

        x2, y2 = interp_x(traj_time), interp_y(traj_time)
        errors = np.sqrt((stitched_x - x2)**2 + (stitched_y - y2)**2)
        errors = errors[~np.isnan(errors)]
        print(f"RMS Error: {np.sqrt(np.mean(errors**2)):.3f} m")

        print(f"Max Error: {np.max(errors):.2f}")
        print(f"95th Percentile: {np.percentile(errors, 95):.2f}")
        print(f"MAE: {np.mean(np.abs(errors)):.2f}")
        plt.show()
        


        # stability analysis
        print("number of outlier windows in window matching:", np.round(outlier_percent,2), "%")
        
        diff_in_shifts = np.diff(all_best_shifts, axis = 0)
        plt.figure(figsize=(15, 8))
        plt.subplot(2,1,1)
        plt.plot(diff_in_shifts[:,0])
        plt.xlabel('number of windows')
        plt.ylabel('shift in map pixels')
        plt.title('X-axis Shift Variation Across Windows')
        plt.subplot(2,1,2)
        plt.plot(diff_in_shifts[:,1])
        plt.xlabel('number of windows')
        plt.ylabel('shift in map pixels')
        plt.title('Y-axis Shift Variation Across Windows')
        plt.tight_layout()
        plt.show()
        
        S = np.asarray(all_best_shifts, dtype=int)
        # Histogram of shifts
        plt.figure()
        plt.hist2d(S[:,0], S[:,1], bins=(min(20, max(5, len(np.unique(S[:,0])))),
                                         min(20, max(5, len(np.unique(S[:,1]))))))
        plt.title("Shift (dx,dy) distribution")
        plt.xlabel("dx"); plt.ylabel("dy")
        plt.colorbar(label="Count"); plt.tight_layout()
        
        # Visualization Requirements
        
        plt.figure(figsize=(10, 5))
        plt.imshow(self.energy_map, cmap='viridis', origin='lower')
        plt.plot(gnss_data['x'], gnss_data['y'], color='blue', label='GNSS')
        plt.legend()
        plt.title("GNSS (Ground Truth) Trajectory")
        
        plt.figure(figsize=(10, 5))
        plt.imshow(self.energy_map, cmap='viridis', origin='lower')
        plt.plot(stitched_x, stitched_y, color='red', label='Matched Trajectory')
        plt.plot(gnss_data['x'], gnss_data['y'], color='blue', label='GNSS')
        plt.legend()
        plt.title("Ground Truth vs Estimated Trajectory")
        
        traj_time = np.array(odom_data['timestamp'][:-1])
        interp_x = interp1d(gnss_data["timestamp"], gnss_data["x"], bounds_error=False, fill_value="extrapolate")
        interp_y = interp1d(gnss_data["timestamp"], gnss_data["y"], bounds_error=False, fill_value="extrapolate")
        
        gnss_interp_x = interp_x(traj_time)
        gnss_interp_y = interp_y(traj_time)
        
        errors = np.sqrt((stitched_x - gnss_interp_x) ** 2 + (stitched_y - gnss_interp_y) ** 2)
        errors = errors[~np.isnan(errors)]
        

        plt.figure()
        plt.plot(errors)
        plt.title("Trajectory Error Over Time")
        plt.xlabel("Time index")
        plt.ylabel("Localization Error [m]")
        plt.grid()
        plt.show()
        
        # Histogram
        plt.figure(figsize=(8, 5))
        plt.hist(errors, bins=30, color='steelblue', edgecolor='black')
        plt.xlabel('Position Error (meters)')
        plt.ylabel('Frequency')
        plt.title('Error Distribution Histogram')
        plt.grid(True, alpha=0.3)
        plt.tight_layout()
        plt.show()
        


def main():
    t0 = time.perf_counter()
    paths = {
        "compass": "data/compass_data.json",
        "odom": "data/odometry_data.json",
        "gnss": "data/gnss_data.json",
        "energy": "data/energy_data.json",
        "map": "data/energy_map.npy"
    }

    sigma_heading = 0.1
    sigma_odom = 0.02
    sigma_energy = 1.0

    data = SensorDataLoader(paths).load()
    compass_heading, dx_rot, dy_rot = OdometryProcessor.rotate_odom(data.compass_data, data.odom_data)
    x_fused, y_fused = OdometryProcessor.fuse_odom_compass(sigma_heading, sigma_odom, compass_heading, dx_rot, dy_rot)

    matcher = EnergyMapMatcher(data.energy_map, data.energy_data)
    x_shifted, y_shifted = matcher.push_into_map(x_fused, y_fused)

    map_size = max(data.energy_map.shape)
    trajectory_length = len(x_shifted)
    levels = [
        {"stride": int(0.05 * map_size), "radius": map_size, "window": trajectory_length},
        {"stride": int(0.02 * map_size), "radius": int(0.1 * map_size), "window": trajectory_length},
        {"stride": int(0.001 * map_size), "radius": int(0.01 * map_size), "window": trajectory_length}
    ]

    x_aligned, y_aligned, _ = matcher.hierarchical_map_matching(x_shifted, y_shifted, data.energy_data["true_energy"], sigma_energy, sigma_odom, levels, (0, 0))
    convergence_time_wall = time.perf_counter() - t0
    print(f"Convergence time (code start until first window match): {convergence_time_wall:.3f} seconds")

    stitcher = TrajectoryStitcher(matcher, levels, sigma_energy, sigma_odom)
    stitched_x, stitched_y, all_best_shifts, outlier_count = stitcher.match_per_window(x_aligned, y_aligned, data.energy_data["true_energy"], data.gnss_data, data.odom_data)
    
    


        
    evaluator = TrajectoryEvaluator(data.energy_map)
    evaluator.metrics_and_plots(stitched_x, stitched_y, data.gnss_data, data.odom_data,all_best_shifts, outlier_count)

if __name__ == "__main__":
    main()
