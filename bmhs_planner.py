import cv2
import numpy as np
import argparse
import math
import heapq
import random
import time
import concurrent.futures

class MapProcessor:
    def __init__(self, map_path, resolution, vehicle_width, vehicle_length, inflation_radius=None):
        self.resolution = resolution
        self.map_img = cv2.imread(map_path, cv2.IMREAD_GRAYSCALE)
        if self.map_img is None:
            raise ValueError(f"Could not read map at {map_path}")
        
        _, self.binary_map = cv2.threshold(self.map_img, 200, 255, cv2.THRESH_BINARY)
        
        inf_rad = inflation_radius if inflation_radius is not None else max(vehicle_width, vehicle_length) / 2.0
        self.inflation_px = int(math.ceil(inf_rad / self.resolution))
        
        if self.inflation_px > 0:
            kernel_size = self.inflation_px * 2 + 1
            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (kernel_size, kernel_size))
            self.inflated_map = cv2.erode(self.binary_map, kernel, iterations=1)
        else:
            self.inflated_map = self.binary_map.copy()
            
        self.height, self.width = self.inflated_map.shape
        self.clearance_map = cv2.distanceTransform(self.inflated_map, cv2.DIST_L2, 5)

    def is_valid(self, x, y):
        ix, iy = int(x), int(y)
        if 0 <= ix < self.width and 0 <= iy < self.height:
            return self.inflated_map[iy, ix] == 255
        return False
    
    def get_random_valid_point(self):
        while True:
            x = random.randint(0, self.width - 1)
            y = random.randint(0, self.height - 1)
            if self.is_valid(x, y):
                theta = random.uniform(-math.pi, math.pi)
                return (x, y, theta)

class VehicleKinematics:
    def __init__(self, v_type, turning_radius, resolution):
        self.v_type = v_type.lower()
        self.tr_px = turning_radius / resolution
        self.resolution = resolution
        self.step_size_px = max(2.0, 0.25 / resolution) # e.g. 5 pixels for 0.05m res
        self.theta_bins = 72
        
    def state_to_grid(self, state):
        x, y, theta = state
        # Wrap theta between 0 and 2*pi
        th = (theta + 2*math.pi) % (2*math.pi)
        th_bin = int(round(th / (2*math.pi / self.theta_bins))) % self.theta_bins
        
        # Bin x and y by a fraction of step size to allow intersection between trees
        bin_size = max(1.0, self.step_size_px * 0.5)
        return (int(x / bin_size), int(y / bin_size), th_bin)

    def get_primitives(self, state, clearance_map, reverse=False):
        x, y, theta = state
        primitives = []
        
        if self.v_type == 'ackermann':
            max_delta_theta = self.step_size_px / self.tr_px if self.tr_px > 0 else 0.5
            steerings = [0, max_delta_theta, -max_delta_theta]
            directions = [1, -1]
            
            for d in directions:
                for steer in steerings:
                    real_d = d if not reverse else -d
                    
                    new_theta = theta + real_d * steer
                    new_theta = (new_theta + math.pi) % (2 * math.pi) - math.pi
                    
                    new_x = x + real_d * self.step_size_px * math.cos(theta + real_d * steer/2.0)
                    new_y = y + real_d * self.step_size_px * math.sin(theta + real_d * steer/2.0)
                    
                    cost = self.step_size_px * self.resolution
                    
                    # Clearance penalty
                    ix, iy = int(new_x), int(new_y)
                    if 0 <= ix < clearance_map.shape[1] and 0 <= iy < clearance_map.shape[0]:
                        clearance = clearance_map[iy, ix] * self.resolution
                        if clearance < 0.5:
                            cost += 1.5 / (clearance + 0.1)
                            
                    if steer != 0:
                        cost += 1.0 * abs(steer)
                    if d == -1:
                        cost += 5.0 * self.step_size_px * self.resolution
                        
                    primitives.append(((new_x, new_y, new_theta), cost))
                    
        else: # differential
            # Composite "turn-then-move" primitives (forward only):
            # 8 forward directions cover all 360°, no backward needed.
            target_headings = [i * math.pi / 4 for i in range(8)]

            for target_theta in target_headings:
                angle_diff = (target_theta - theta + math.pi) % (2 * math.pi) - math.pi

                real_d = 1 if not reverse else -1

                new_x = x + real_d * self.step_size_px * math.cos(target_theta)
                new_y = y + real_d * self.step_size_px * math.sin(target_theta)

                cost = self.step_size_px * self.resolution

                if abs(angle_diff) > 0.01:
                    cost += 2.0 + 3.0 * abs(angle_diff)

                ix, iy = int(new_x), int(new_y)
                if 0 <= ix < clearance_map.shape[1] and 0 <= iy < clearance_map.shape[0]:
                    clearance = clearance_map[iy, ix] * self.resolution
                    if clearance < 0.5:
                        cost += 1.5 / (clearance + 0.1)

                new_theta = (target_theta + math.pi) % (2 * math.pi) - math.pi
                primitives.append(((new_x, new_y, new_theta), cost))
                
        return primitives

def euclidean_dist(s1, s2, res):
    return math.hypot(s1[0]-s2[0], s1[1]-s2[1]) * res

def compute_2d_heuristic(map_proc, target_px):
    inflated_map = map_proc.inflated_map
    clearance_map = map_proc.clearance_map
    res = map_proc.resolution
    
    h, w = inflated_map.shape
    dist = np.full((h, w), np.inf)
    tx, ty = int(target_px[0]), int(target_px[1])
    
    if not (0 <= tx < w and 0 <= ty < h) or inflated_map[ty, tx] != 255:
        return dist
        
    dist[ty, tx] = 0.0
    pq = [(0.0, tx, ty)]
    
    dirs = [(1,0,1), (-1,0,1), (0,1,1), (0,-1,1),
            (1,1,1.414), (-1,1,1.414), (1,-1,1.414), (-1,-1,1.414)]
            
    while pq:
        d, x, y = heapq.heappop(pq)
        if d > dist[y, x]:
            continue
            
        for dx, dy, cost in dirs:
            nx, ny = x + dx, y + dy
            if 0 <= nx < w and 0 <= ny < h and inflated_map[ny, nx] == 255:
                clearance = clearance_map[ny, nx] * res
                penalty = 1.0 / (clearance + 0.1) if clearance < 1.0 else 0.0
                new_d = d + cost + penalty
                if new_d < dist[ny, nx]:
                    dist[ny, nx] = new_d
                    heapq.heappush(pq, (new_d, nx, ny))
                    
    return dist * res

class BMHS:
    def __init__(self, map_proc, kinematics):
        self.map_proc = map_proc
        self.kin = kinematics
        self.searched_nodes_f = []
        self.searched_nodes_b = []

    def check_intersection(self, grid, closed_other):
        xg, yg, thg = grid
        
        if self.kin.v_type == 'differential':
            # For differential, orientation doesn't matter for intersection
            # because the robot can turn in place to match the orientation.
            theta_offsets = range(self.kin.theta_bins)
        else:
            # For ackermann, orientations must closely match for a smooth path
            theta_offsets = [-2, -1, 0, 1, 2]
            
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                for dth in theta_offsets:
                    chk_th = (thg + dth) % self.kin.theta_bins
                    chk_grid = (xg + dx, yg + dy, chk_th)
                    if chk_grid in closed_other:
                        return chk_grid
        return None

    def plan(self, start, goal):
        print("Precomputing 2D heuristics (this may take a few seconds)...")
        
        t0 = time.time()
        
        with concurrent.futures.ProcessPoolExecutor(max_workers=2) as executor:
            future_goal = executor.submit(compute_2d_heuristic, self.map_proc, goal)
            future_start = executor.submit(compute_2d_heuristic, self.map_proc, start)
            
            h2d_goal = future_goal.result()
            t1 = time.time()
            print(f"[Timer] Heuristic 1 (Goal) parallel calculation complete: {t1 - t0:.4f}s")
            
            h2d_start = future_start.result()
            t2 = time.time()
            print(f"[Timer] Heuristic 2 (Start) parallel calculation complete: {t2 - t1:.4f}s")
            print(f"[Timer] Total Heuristic calculation time: {t2 - t0:.4f}s")
        
        open_f = []
        open_b = []
        
        closed_f = {}
        closed_b = {}
        
        came_from_f = {}
        came_from_b = {}
        
        g_f = {self.kin.state_to_grid(start): 0}
        g_b = {self.kin.state_to_grid(goal): 0}
        
        # Multi-heuristic simple combination (f = g + w1*h1 + w2*h2)
        # We just use a slightly weighted heuristic to speed it up like WA*
        weight = 1.5
        
        start_ix, start_iy = int(start[0]), int(start[1])
        goal_ix, goal_iy = int(goal[0]), int(goal[1])
        
        h_f = h2d_goal[start_iy, start_ix] if 0 <= start_iy < self.map_proc.height and 0 <= start_ix < self.map_proc.width else euclidean_dist(start, goal, self.map_proc.resolution)
        heapq.heappush(open_f, (weight * h_f, 0, start))
        came_from_f[self.kin.state_to_grid(start)] = (None, start)
        
        h_b = h2d_start[goal_iy, goal_ix] if 0 <= goal_iy < self.map_proc.height and 0 <= goal_ix < self.map_proc.width else euclidean_dist(goal, start, self.map_proc.resolution)
        heapq.heappush(open_b, (weight * h_b, 0, goal))
        came_from_b[self.kin.state_to_grid(goal)] = (None, goal)
        
        intersect_node_grid_f = None
        intersect_node_grid_b = None
        intersect_state_f = None
        intersect_state_b = None
        
        max_iters = 500000
        iters = 0
        kin_calc_time = 0.0
        
        t_search_start = time.time()
        
        while open_f and open_b and iters < max_iters:
            iters += 1
            
            # Expand Forward
            if open_f:
                _, curr_g_f, curr_state_f = heapq.heappop(open_f)
                grid_f = self.kin.state_to_grid(curr_state_f)
                
                if grid_f in closed_f:
                    pass
                else:
                    closed_f[grid_f] = curr_state_f
                    self.searched_nodes_f.append(curr_state_f)
                    
                    chk_res = self.check_intersection(grid_f, closed_b)
                    if chk_res is not None:
                        intersect_node_grid_f = grid_f
                        intersect_node_grid_b = chk_res
                        intersect_state_f = curr_state_f
                        intersect_state_b = closed_b[chk_res]
                        break
                        
                    tk0 = time.time()
                    primitives_f = self.kin.get_primitives(curr_state_f, self.map_proc.clearance_map, reverse=False)
                    kin_calc_time += time.time() - tk0
                    
                    for next_state, step_cost in primitives_f:
                        if not self.map_proc.is_valid(next_state[0], next_state[1]):
                            continue
                            
                        next_grid = self.kin.state_to_grid(next_state)
                        new_g = curr_g_f + step_cost
                        
                        if next_grid not in g_f or new_g < g_f[next_grid]:
                            g_f[next_grid] = new_g
                            nix, niy = int(next_state[0]), int(next_state[1])
                            if 0 <= nix < self.map_proc.width and 0 <= niy < self.map_proc.height:
                                h = h2d_goal[niy, nix]
                            else:
                                h = euclidean_dist(next_state, goal, self.map_proc.resolution)
                            
                            f = new_g + weight * h
                            heapq.heappush(open_f, (f, new_g, next_state))
                            came_from_f[next_grid] = (grid_f, next_state)

            # Expand Backward
            if open_b:
                _, curr_g_b, curr_state_b = heapq.heappop(open_b)
                grid_b = self.kin.state_to_grid(curr_state_b)
                
                if grid_b in closed_b:
                    pass
                else:
                    closed_b[grid_b] = curr_state_b
                    self.searched_nodes_b.append(curr_state_b)
                    
                    chk_res = self.check_intersection(grid_b, closed_f)
                    if chk_res is not None:
                        intersect_node_grid_f = chk_res
                        intersect_node_grid_b = grid_b
                        intersect_state_f = closed_f[chk_res]
                        intersect_state_b = curr_state_b
                        break
                        
                    tk0 = time.time()
                    primitives_b = self.kin.get_primitives(curr_state_b, self.map_proc.clearance_map, reverse=True)
                    kin_calc_time += time.time() - tk0
                    
                    for next_state, step_cost in primitives_b:
                        if not self.map_proc.is_valid(next_state[0], next_state[1]):
                            continue
                            
                        next_grid = self.kin.state_to_grid(next_state)
                        new_g = curr_g_b + step_cost
                        
                        if next_grid not in g_b or new_g < g_b[next_grid]:
                            g_b[next_grid] = new_g
                            nix, niy = int(next_state[0]), int(next_state[1])
                            if 0 <= nix < self.map_proc.width and 0 <= niy < self.map_proc.height:
                                h = h2d_start[niy, nix]
                            else:
                                h = euclidean_dist(next_state, start, self.map_proc.resolution)
                                
                            f = new_g + weight * h
                            heapq.heappush(open_b, (f, new_g, next_state))
                            came_from_b[next_grid] = (grid_b, next_state)

        t_search_end = time.time()
        print(f"[Timer] Forward and backward search converge time: {t_search_end - t_search_start:.4f}s")
        print(f"[Timer] Accumulated kinematic cost calculation time: {kin_calc_time:.4f}s")

        if intersect_node_grid_f is None:
            print(f"Failed to find a path. Iterations: {iters}")
            return None
            
        print(f"Path found. Intersected at grids {intersect_node_grid_f} & {intersect_node_grid_b} after {iters} iterations.")
        
        # Reconstruct path
        path_f = []
        curr = intersect_node_grid_f
        while curr is not None and curr in came_from_f:
            parent_grid, state = came_from_f[curr]
            path_f.append(state)
            curr = parent_grid
        path_f.reverse()
        
        path_b = []
        curr = intersect_node_grid_b
        # Skip the intersection node itself for backward part so we don't duplicate
        if curr in came_from_b:
            parent_grid, state = came_from_b[curr]
            # Since the nodes didn't match perfectly, include the state of intersection for b
            path_b.append(intersect_state_b)
            curr = parent_grid
            
        while curr is not None and curr in came_from_b:
            parent_grid, state = came_from_b[curr]
            path_b.append(state)
            curr = parent_grid
            
        raw_path = path_f + path_b
        simplified = self.simplify_path(raw_path)
        print(f"Path simplified: {len(raw_path)} waypoints -> {len(simplified)} waypoints")

        if self.kin.v_type == 'ackermann' and len(simplified) > 2:
            smoothed = self.smooth_path_ackermann(simplified)
            print(f"Path smoothed for ackermann: {len(simplified)} waypoints -> {len(smoothed)} waypoints")
            return smoothed

        return simplified

    def smooth_path_ackermann(self, path):
        """Replace sharp corners with smooth quadratic Bezier arcs."""
        if len(path) <= 2:
            return path
        turning_radius_px = self.kin.tr_px
        smoothed = [path[0]]
        for i in range(1, len(path) - 1):
            prev = path[i - 1]
            curr = path[i]
            next_pt = path[i + 1]
            dx_in = curr[0] - prev[0]
            dy_in = curr[1] - prev[1]
            dx_out = next_pt[0] - curr[0]
            dy_out = next_pt[1] - curr[1]
            len_in = math.hypot(dx_in, dy_in)
            len_out = math.hypot(dx_out, dy_out)
            if len_in < 1e-6 or len_out < 1e-6:
                smoothed.append(curr)
                continue
            offset = min(turning_radius_px * 3, len_in * 0.4, len_out * 0.4)
            entry_x = curr[0] - (dx_in / len_in) * offset
            entry_y = curr[1] - (dy_in / len_in) * offset
            exit_x = curr[0] + (dx_out / len_out) * offset
            exit_y = curr[1] + (dy_out / len_out) * offset
            theta_in = math.atan2(dy_in, dx_in)
            smoothed.append((entry_x, entry_y, theta_in))
            num_samples = max(10, int(offset / 2))
            curve_valid = True
            curve_points = []
            for t_idx in range(1, num_samples + 1):
                t = t_idx / num_samples
                x = (1 - t) ** 2 * entry_x + 2 * (1 - t) * t * curr[0] + t ** 2 * exit_x
                y = (1 - t) ** 2 * entry_y + 2 * (1 - t) * t * curr[1] + t ** 2 * exit_y
                tx = 2 * (1 - t) * (curr[0] - entry_x) + 2 * t * (exit_x - curr[0])
                ty = 2 * (1 - t) * (curr[1] - entry_y) + 2 * t * (exit_y - curr[1])
                theta = math.atan2(ty, tx)
                if not self.map_proc.is_valid(x, y):
                    curve_valid = False
                    break
                ix, iy = int(x), int(y)
                if 0 <= ix < self.map_proc.width and 0 <= iy < self.map_proc.height:
                    clearance = self.map_proc.clearance_map[iy, ix] * self.map_proc.resolution
                    if clearance < 0.2:
                        curve_valid = False
                        break
                curve_points.append((x, y, theta))
            if curve_valid and curve_points:
                smoothed.extend(curve_points)
            else:
                smoothed.append(curr)
        smoothed.append(path[-1])
        return smoothed

    def has_line_of_sight(self, x0, y0, x1, y1, min_clearance_m=0.3):
        """Bresenham line check with clearance enforcement."""
        ix0, iy0 = int(x0), int(y0)
        ix1, iy1 = int(x1), int(y1)
        clearance_map = self.map_proc.clearance_map
        res = self.map_proc.resolution
        dx = abs(ix1 - ix0)
        dy = abs(iy1 - iy0)
        sx = 1 if ix0 < ix1 else -1
        sy = 1 if iy0 < iy1 else -1
        err = dx - dy
        while True:
            if not self.map_proc.is_valid(ix0, iy0):
                return False
            if 0 <= ix0 < clearance_map.shape[1] and 0 <= iy0 < clearance_map.shape[0]:
                if clearance_map[iy0, ix0] * res < min_clearance_m:
                    return False
            if ix0 == ix1 and iy0 == iy1:
                break
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                ix0 += sx
            if e2 < dx:
                err += dx
                iy0 += sy
        return True

    def simplify_path(self, path):
        """Remove unnecessary waypoints by checking line-of-sight with clearance."""
        if not path or len(path) <= 2:
            return path
        simplified = [path[0]]
        i = 0
        while i < len(path) - 1:
            farthest = i + 1
            for j in range(len(path) - 1, i, -1):
                if self.has_line_of_sight(path[i][0], path[i][1], path[j][0], path[j][1]):
                    farthest = j
                    break
            dx = path[farthest][0] - path[i][0]
            dy = path[farthest][1] - path[i][1]
            heading = math.atan2(dy, dx)
            simplified[-1] = (simplified[-1][0], simplified[-1][1], heading)
            simplified.append((path[farthest][0], path[farthest][1], path[farthest][2]))
            i = farthest
        return simplified

def visualize(map_proc, searched_f, searched_b, path, start, goal, save_opt=False):
    # Create color map
    vis_img = cv2.cvtColor(map_proc.map_img, cv2.COLOR_GRAY2BGR)
    
    # Draw inflated obstacles in red (overlay)
    inflated_overlay = np.zeros_like(vis_img)
    inflated_overlay[map_proc.inflated_map == 0] = [0, 0, 255] # Red
    cv2.addWeighted(vis_img, 0.7, inflated_overlay, 0.3, 0, vis_img)
    
    # Draw searched nodes F (cyan)
    for state in searched_f:
        cv2.circle(vis_img, (int(state[0]), int(state[1])), 1, (255, 255, 0), -1)
        
    # Draw searched nodes B (magenta)
    for state in searched_b:
        cv2.circle(vis_img, (int(state[0]), int(state[1])), 1, (255, 0, 255), -1)
        
    # Draw path (green)
    if path:
        for i in range(len(path)-1):
            pt1 = (int(path[i][0]), int(path[i][1]))
            pt2 = (int(path[i+1][0]), int(path[i+1][1]))
            cv2.line(vis_img, pt1, pt2, (0, 255, 0), 2)
            
    # Draw start and goal
    cv2.circle(vis_img, (int(start[0]), int(start[1])), 5, (255, 0, 0), -1) # Blue start
    cv2.circle(vis_img, (int(goal[0]), int(goal[1])), 5, (0, 0, 255), -1)   # Red goal
    
    # Add orientation vectors
    def draw_arrow(pt, theta, color):
        p1 = (int(pt[0]), int(pt[1]))
        p2 = (int(pt[0] + 15*math.cos(theta)), int(pt[1] + 15*math.sin(theta)))
        cv2.arrowedLine(vis_img, p1, p2, color, 2, tipLength=0.3)
        
    draw_arrow(start, start[2], (255, 0, 0))
    draw_arrow(goal, goal[2], (0, 0, 255))
    
    cv2.imshow("BMHS Path Planner", vis_img)
    print("Press any key to close the visualization...")
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    
    if save_opt:
        cv2.imwrite("planned_path.png", vis_img)
        print("Saved visualization to planned_path.png")

def main():
    parser = argparse.ArgumentParser(description="BMHS Path Planner")
    parser.add_argument('--map', type=str, default='map_proj.pgm', help='Path to PGM map')
    parser.add_argument('--res', type=float, default=0.05, help='Map resolution (m/pixel)')
    parser.add_argument('--vtype', type=str, default='differential', choices=['ackermann', 'differential'], help='Vehicle kinematics type')
    parser.add_argument('--width', type=float, default=1.0, help='Vehicle width (m)')
    parser.add_argument('--length', type=float, default=1.0, help='Vehicle length (m)')
    parser.add_argument('--inflation', type=float, default=0.3, help='Obstacle inflation radius (m)')
    
    parser.add_argument('--auto', action='store_true', help='Auto generate random start and goal')
    parser.add_argument('--start', type=float, nargs=3, help='Start state (x, y, theta_rad) in meters')
    parser.add_argument('--goal', type=float, nargs=3, help='Goal state (x, y, theta_rad) in meters')
    parser.add_argument('--save', action='store_true', help='Save output image')
    
    args = parser.parse_args()
    
    t0 = time.time()
    map_proc = MapProcessor(args.map, args.res, args.width, args.length, args.inflation)
    print(f"[Timer] Map loading & processing: {time.time() - t0:.4f}s")
    
    if args.vtype == 'ackermann':
        # Assume a standard max steering angle of 30 degrees (approx 0.523 rad)
        # R = L / tan(max_steer)
        tr = args.length / math.tan(math.radians(30))
        print(f"Computed minimum turning radius for ackermann: {tr:.2f}m")
    else:
        tr = 0.0
        
    kinematics = VehicleKinematics(args.vtype, tr, args.res)
    
    if args.auto:
        start_px = map_proc.get_random_valid_point()
        goal_px = map_proc.get_random_valid_point()
    else:
        if args.start and args.goal:
            start_px = (args.start[0]/args.res, args.start[1]/args.res, args.start[2])
            goal_px = (args.goal[0]/args.res, args.goal[1]/args.res, args.goal[2])
        else:
            print("No start/goal provided. Using --auto to generate.")
            start_px = map_proc.get_random_valid_point()
            goal_px = map_proc.get_random_valid_point()
            
    if not map_proc.is_valid(start_px[0], start_px[1]):
        print("Error: Start position is invalid (collision or out of bounds).")
        return
    if not map_proc.is_valid(goal_px[0], goal_px[1]):
        print("Error: Goal position is invalid (collision or out of bounds).")
        return
        
    print(f"Start: ({start_px[0]*args.res:.2f}m, {start_px[1]*args.res:.2f}m, {start_px[2]:.2f}rad)")
    print(f"Goal: ({goal_px[0]*args.res:.2f}m, {goal_px[1]*args.res:.2f}m, {goal_px[2]:.2f}rad)")
    
    planner = BMHS(map_proc, kinematics)
    print("Planning...")
    path = planner.plan(start_px, goal_px)
    
    visualize(map_proc, planner.searched_nodes_f, planner.searched_nodes_b, path, start_px, goal_px, args.save)

if __name__ == '__main__':
    main()
