"""USGS DEM Global Planner.

Loads pre-downloaded USGS Digital Elevation Model GeoTIFF tiles for the
URC competition area (Mars Desert Research Station, Hanksville UT) and
runs an A* search over a traversability grid derived from slope + elevation
gradient. WVU Team Mountaineers' top-autonomy-scoring stack uses this same
pre-loaded DEM approach instead of relying on online global mapping in
feature-poor desert terrain.

Inputs (consumed):
  /odometry/filtered_map  (nav_msgs/Odometry) — current rover global pose
  /goal_pose              (geometry_msgs/PoseStamped) — target pose

Outputs (published):
  /plan                   (nav_msgs/Path) — global path in map frame
  /traversability         (nav_msgs/OccupancyGrid) — debug visualization

Parameters:
  dem_directory       — folder containing .tif tiles
  utm_zone            — UTM zone for MDRS (12 North)
  utm_north           — True (northern hemisphere)
  max_slope_deg       — slope above which terrain is non-traversable
  resolution          — planner grid resolution (m)
  inflation_radius    — rover footprint inflation (m)
"""

import math
import os
import heapq
from typing import List, Tuple, Optional

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path, OccupancyGrid, MapMetaData
from std_msgs.msg import Header

try:
    import rasterio
    from rasterio.merge import merge as rio_merge
    RASTERIO_AVAILABLE = True
except ImportError:
    RASTERIO_AVAILABLE = False
    rasterio = None

try:
    from pyproj import Transformer
    PYPROJ_AVAILABLE = True
except ImportError:
    PYPROJ_AVAILABLE = False
    Transformer = None


def slope_from_dem(dem: np.ndarray, cell_size: float) -> np.ndarray:
    """Slope in degrees per cell, computed via Sobel-style central differences."""
    dz_dx = np.zeros_like(dem)
    dz_dy = np.zeros_like(dem)
    dz_dx[:, 1:-1] = (dem[:, 2:] - dem[:, :-2]) / (2.0 * cell_size)
    dz_dy[1:-1, :] = (dem[2:, :] - dem[:-2, :]) / (2.0 * cell_size)
    grad = np.sqrt(dz_dx ** 2 + dz_dy ** 2)
    return np.degrees(np.arctan(grad))


def astar(
    grid: np.ndarray,
    start: Tuple[int, int],
    goal: Tuple[int, int],
    cost_scale: float = 1.0,
) -> Optional[List[Tuple[int, int]]]:
    """8-connected A* over a 2D cost grid. grid[r, c] >= 0; <0 is blocked."""
    h, w = grid.shape
    if not (0 <= start[0] < h and 0 <= start[1] < w):
        return None
    if not (0 <= goal[0] < h and 0 <= goal[1] < w):
        return None

    def heur(a, b):
        return math.hypot(a[0] - b[0], a[1] - b[1])

    open_set = [(heur(start, goal), 0.0, start, None)]
    came = {}
    g_score = {start: 0.0}

    neighbors = [
        (-1, -1, math.sqrt(2)), (-1, 0, 1.0), (-1, 1, math.sqrt(2)),
        (0, -1, 1.0),                         (0, 1, 1.0),
        (1, -1, math.sqrt(2)),  (1, 0, 1.0),  (1, 1, math.sqrt(2)),
    ]

    while open_set:
        _, g, current, parent = heapq.heappop(open_set)
        if current in came and came[current] is not None and g >= g_score.get(current, float('inf')):
            continue
        if parent is not None:
            came[current] = parent
        else:
            came[current] = None
        if current == goal:
            path = [current]
            while came[current] is not None:
                current = came[current]
                path.append(current)
            return list(reversed(path))

        cr, cc = current
        for dr, dc, step in neighbors:
            nr, nc = cr + dr, cc + dc
            if not (0 <= nr < h and 0 <= nc < w):
                continue
            cost = grid[nr, nc]
            if cost < 0:
                continue
            tentative = g + step * (1.0 + cost * cost_scale)
            if tentative < g_score.get((nr, nc), float('inf')):
                g_score[(nr, nc)] = tentative
                f = tentative + heur((nr, nc), goal)
                heapq.heappush(open_set, (f, tentative, (nr, nc), current))
    return None


class DemGlobalPlanner(Node):
    def __init__(self):
        super().__init__('dem_global_planner')

        self.declare_parameter(
            'dem_directory', os.path.expanduser('~/deimos_data/mdrs_dem')
        )
        self.declare_parameter('utm_zone', 12)
        self.declare_parameter('utm_north', True)
        self.declare_parameter('max_slope_deg', 25.0)
        self.declare_parameter('inflation_radius_m', 0.7)
        self.declare_parameter('plan_topic', '/plan')
        self.declare_parameter('odom_topic', '/odometry/filtered_map')
        self.declare_parameter('goal_topic', '/goal_pose')
        self.declare_parameter('downsample_path_step', 2)

        self.dem: Optional[np.ndarray] = None
        self.dem_transform = None
        self.dem_origin_x = 0.0
        self.dem_origin_y = 0.0
        self.dem_resolution = 1.0
        self.cost_grid: Optional[np.ndarray] = None
        self._latest_pose: Optional[PoseStamped] = None
        self._utm_to_latlon = None
        self._latlon_to_utm = None

        if PYPROJ_AVAILABLE:
            zone = int(self.get_parameter('utm_zone').value)
            north = bool(self.get_parameter('utm_north').value)
            epsg = 32600 + zone if north else 32700 + zone
            self._latlon_to_utm = Transformer.from_crs(
                'EPSG:4326', f'EPSG:{epsg}', always_xy=True
            )
            self._utm_to_latlon = Transformer.from_crs(
                f'EPSG:{epsg}', 'EPSG:4326', always_xy=True
            )

        self._load_dem()

        odom_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        latched = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.odom_sub = self.create_subscription(
            Odometry, self.get_parameter('odom_topic').value, self._odom_cb, odom_qos
        )
        self.goal_sub = self.create_subscription(
            PoseStamped, self.get_parameter('goal_topic').value, self._goal_cb, 10
        )
        self.plan_pub = self.create_publisher(
            Path, self.get_parameter('plan_topic').value, latched
        )
        self.cost_pub = self.create_publisher(OccupancyGrid, '/traversability', latched)
        self._publish_cost_grid()

    def _load_dem(self):
        if not RASTERIO_AVAILABLE:
            self.get_logger().error(
                'rasterio not installed — install via `pip install rasterio` '
                'on the rover before competition.'
            )
            return

        dem_dir = self.get_parameter('dem_directory').value
        if not os.path.isdir(dem_dir):
            self.get_logger().warn(
                f'DEM directory {dem_dir} not found. Planner will fall back '
                f'to flat-traversability mode.'
            )
            return

        tifs = [
            os.path.join(dem_dir, f) for f in os.listdir(dem_dir)
            if f.lower().endswith(('.tif', '.tiff'))
        ]
        if not tifs:
            self.get_logger().warn(
                f'No GeoTIFF tiles in {dem_dir}; falling back to flat mode.'
            )
            return

        srcs = [rasterio.open(f) for f in tifs]
        merged, transform = rio_merge(srcs)
        self.dem = merged[0].astype(np.float32)
        self.dem_transform = transform
        self.dem_resolution = float(transform.a)
        self.dem_origin_x = float(transform.c)
        self.dem_origin_y = float(transform.f)
        for s in srcs:
            s.close()

        slope = slope_from_dem(self.dem, self.dem_resolution)
        max_slope = float(self.get_parameter('max_slope_deg').value)
        cost = np.zeros_like(slope)
        cost[slope > max_slope] = -1.0
        cost[slope <= max_slope] = (slope[slope <= max_slope] / max_slope)
        self.cost_grid = cost

        self.get_logger().info(
            f'Loaded DEM: shape={self.dem.shape}, resolution={self.dem_resolution} m, '
            f'origin=({self.dem_origin_x:.1f}, {self.dem_origin_y:.1f}). '
            f'Traversable cells: {(cost >= 0).sum()} / {cost.size}'
        )

    def _publish_cost_grid(self):
        if self.cost_grid is None:
            return
        grid = OccupancyGrid()
        grid.header = Header()
        grid.header.frame_id = 'map'
        grid.header.stamp = self.get_clock().now().to_msg()
        meta = MapMetaData()
        meta.resolution = float(self.dem_resolution)
        meta.width = int(self.cost_grid.shape[1])
        meta.height = int(self.cost_grid.shape[0])
        meta.origin.position.x = float(self.dem_origin_x)
        meta.origin.position.y = float(self.dem_origin_y)
        meta.origin.position.z = 0.0
        grid.info = meta
        viz = self.cost_grid.copy()
        viz_int = np.where(viz < 0, 100, (viz * 99).clip(0, 99).astype(np.int8))
        grid.data = viz_int.flatten().tolist()
        self.cost_pub.publish(grid)

    def _odom_cb(self, msg: Odometry):
        ps = PoseStamped()
        ps.header = msg.header
        ps.pose = msg.pose.pose
        self._latest_pose = ps

    def _goal_cb(self, goal: PoseStamped):
        if self._latest_pose is None:
            self.get_logger().warn('No odometry yet — cannot plan.')
            return

        if self.cost_grid is None:
            # Fallback: straight-line plan
            path = Path()
            path.header = goal.header
            path.poses = [self._latest_pose, goal]
            self.plan_pub.publish(path)
            return

        sx = self._latest_pose.pose.position.x
        sy = self._latest_pose.pose.position.y
        gx = goal.pose.position.x
        gy = goal.pose.position.y

        start_idx = self._world_to_grid(sx, sy)
        goal_idx = self._world_to_grid(gx, gy)
        if start_idx is None or goal_idx is None:
            self.get_logger().warn('Start or goal outside DEM extent.')
            return

        path_idx = astar(self.cost_grid, start_idx, goal_idx)
        if path_idx is None:
            self.get_logger().warn('No traversable path to goal.')
            return

        step = max(1, int(self.get_parameter('downsample_path_step').value))
        downsampled = path_idx[::step]
        if downsampled[-1] != path_idx[-1]:
            downsampled.append(path_idx[-1])

        path = Path()
        path.header.frame_id = 'map'
        path.header.stamp = self.get_clock().now().to_msg()

        for r, c in downsampled:
            x, y = self._grid_to_world(r, c)
            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position.x = x
            ps.pose.position.y = y
            ps.pose.orientation.w = 1.0
            path.poses.append(ps)

        self.plan_pub.publish(path)
        self.get_logger().info(
            f'Plan published: {len(path.poses)} poses, '
            f'{len(path_idx)} grid cells'
        )

    def _world_to_grid(self, x: float, y: float) -> Optional[Tuple[int, int]]:
        if self.dem is None:
            return None
        c = int((x - self.dem_origin_x) / self.dem_resolution)
        r = int((self.dem_origin_y - y) / abs(self.dem_resolution))
        h, w = self.cost_grid.shape
        if 0 <= r < h and 0 <= c < w:
            return (r, c)
        return None

    def _grid_to_world(self, r: int, c: int) -> Tuple[float, float]:
        x = self.dem_origin_x + (c + 0.5) * self.dem_resolution
        y = self.dem_origin_y - (r + 0.5) * abs(self.dem_resolution)
        return x, y


def main(args=None):
    rclpy.init(args=args)
    node = DemGlobalPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
