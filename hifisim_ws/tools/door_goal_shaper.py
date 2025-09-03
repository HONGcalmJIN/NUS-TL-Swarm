#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud2, PointField
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
import argparse
import json
import math
import struct
from typing import List, Tuple, Optional


def read_signposts_from_json(json_path: str, name_filter: str = "", limit: int = 0) -> List[Tuple[float, float, float, Optional[float]]]:
	with open(json_path, 'r') as f:
		data = json.load(f)
	sps = []
	for sp in data.get('Signposts', []):
		name = sp.get('ObjName', '')
		if name_filter and (name_filter not in name):
			continue
		tds = sp.get('TransformDatas', [])
		if not tds:
			continue
		pos = tds[0].get('Pos', {})
		eul = tds[0].get('Eul', {})  # degrees if present
		# 兼容大小写键
		def g(d, *keys, default=0.0):
			for k in keys:
				if k in d:
					return d[k]
			return default
		x = float(g(pos, 'x', 'X', default=0.0))
		y = float(g(pos, 'y', 'Y', default=0.0))
		z = float(g(pos, 'z', 'Z', default=0.0))
		# Unity EUN(x_east,y_up,z_north) -> NWU(x_north,y_west,z_up) = (z, -x, y)
		wx, wy, wz = float(z), float(-x), float(y)
		yaw_deg = None
		if ('z' in eul) or ('Z' in eul):
			# 与 mission_runner 保持：yaw_deg = -Eul.z
			yaw_deg = float(-g(eul, 'z', 'Z', default=0.0))
		sps.append((wx, wy, wz, yaw_deg))
	if limit > 0:
		sps = sps[:limit]
	return sps


def quaternion_to_yaw(qx: float, qy: float, qz: float, qw: float) -> float:
	# yaw from quaternion (Z rotation), radians
	sinr_cosp = 2.0 * (qw * qz + qx * qy)
	cosr_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
	return math.atan2(sinr_cosp, cosr_cosp)


def normalize2(vx: float, vy: float) -> Tuple[float, float]:
	l = math.hypot(vx, vy)
	if l < 1e-9:
		return 1.0, 0.0
	return vx / l, vy / l


def rot2d(vx: float, vy: float, deg: float) -> Tuple[float, float]:
	t = math.radians(deg)
	ct, st = math.cos(t), math.sin(t)
	return (ct * vx - st * vy, st * vx + ct * vy)


def build_pose_stamped(x: float, y: float, z: float, frame: str = 'world') -> PoseStamped:
	msg = PoseStamped()
	msg.header.frame_id = frame
	msg.pose.position.x = float(x)
	msg.pose.position.y = float(y)
	msg.pose.position.z = float(z)
	msg.pose.orientation.w = 1.0
	return msg


class DoorGoalShaper(Node):
	def __init__(self,
				 cloud_topic: str,
				 odom_topic: str,
				 goal_out: str,
				 trigger_topic: str,
				 json_path: Optional[str],
				 name_filter: str,
				 limit: int,
				 sector_deg: float,
				 radius: float,
				 safe_dist: float,
				 delta_in: float,
				 delta_post: float,
				 backoff_pre: float,
				 sample_count: int,
				 stride: int,
				 altitude: Optional[float],
				 fixed_left_deg: Optional[float]):
		super().__init__('door_goal_shaper')
		self.cloud_topic = cloud_topic
		self.odom_topic = odom_topic
		self.goal_out = goal_out
		self.trigger_topic = trigger_topic
		self.sector_deg = float(sector_deg)
		self.radius = float(radius)
		self.safe_dist = float(safe_dist)
		self.delta_in = float(delta_in)
		self.delta_post = float(delta_post)
		self.backoff_pre = float(backoff_pre)
		self.sample_count = int(sample_count)
		self.stride = max(1, int(stride))
		self.altitude = altitude
		self.fixed_left_deg = fixed_left_deg

		self.goal_pub = self.create_publisher(PoseStamped, self.goal_out, 10)
		marker_qos = QoSProfile(
			depth=1,
			reliability=ReliabilityPolicy.RELIABLE,
			durability=DurabilityPolicy.TRANSIENT_LOCAL,
			history=HistoryPolicy.KEEP_LAST,
		)
		self.marker_pub = self.create_publisher(MarkerArray, 'door_goal_shaper/markers', marker_qos)

		# Sensor-like QoS for cloud
		sensor_qos = QoSProfile(
			depth=10,
			reliability=ReliabilityPolicy.BEST_EFFORT,
			durability=DurabilityPolicy.VOLATILE,
			history=HistoryPolicy.KEEP_LAST,
		)
		self.cloud_sub = self.create_subscription(PointCloud2, self.cloud_topic, self.cb_cloud, sensor_qos)
		self.odom_sub = self.create_subscription(Odometry, self.odom_topic, self.cb_odom, 10)
		self.trigger_sub = None
		self.triggered = False
		if self.trigger_topic:
			self.trigger_sub = self.create_subscription(Bool, self.trigger_topic, self.cb_trigger, 10)

		self.cloud_points: Optional[List[Tuple[float, float, float]]] = None
		self.uav_pos: Optional[Tuple[float, float, float]] = None

		self.signposts: List[Tuple[float, float, float, Optional[float]]] = []
		if json_path:
			self.signposts = read_signposts_from_json(json_path, name_filter, limit)
			self.get_logger().info(f"Loaded {len(self.signposts)} signposts from JSON")
		if not self.signposts:
			self.get_logger().warn("No signposts loaded; provide --json or pass P_sp some other way.")

		self.timer = self.create_timer(0.5, self.tick)

	def cb_trigger(self, msg: Bool):
		self.triggered = bool(msg.data)

	def cb_cloud(self, msg: PointCloud2):
		# Parse XYZ only; downsample by stride
		try:
			offset = {f.name: f.offset for f in msg.fields}
			if not all(k in offset for k in ('x', 'y', 'z')):
				return
			pts = []
			step = msg.point_step
			data = msg.data
			for i in range(0, msg.width * step, step * self.stride):
				x = struct.unpack_from('f', data, i + offset['x'])[0]
				y = struct.unpack_from('f', data, i + offset['y'])[0]
				z = struct.unpack_from('f', data, i + offset['z'])[0]
				pts.append((x, y, z))
			self.cloud_points = pts
		except Exception:
			pass

	def cb_odom(self, msg: Odometry):
		self.uav_pos = (
			msg.pose.pose.position.x,
			msg.pose.pose.position.y,
			msg.pose.pose.position.z,
		)

	def publish_markers(self, P_sp: Tuple[float, float, float], candidates: List[Tuple[float, float, float]], chosen: Optional[Tuple[float, float, float]], color_choice=(0.2, 1.0, 0.2)):
		arr = MarkerArray()
		# Signpost center
		sp = Marker()
		sp.header.frame_id = 'world'
		sp.type = Marker.SPHERE
		sp.action = Marker.ADD
		sp.scale.x = sp.scale.y = sp.scale.z = 0.15
		sp.color.r = 1.0
		sp.color.g = 1.0
		sp.color.b = 0.0
		sp.color.a = 0.9
		sp.id = 0
		sp.pose.position.x, sp.pose.position.y, sp.pose.position.z = P_sp
		arr.markers.append(sp)
		# Candidates
		m = Marker()
		m.header.frame_id = 'world'
		m.type = Marker.SPHERE_LIST
		m.action = Marker.ADD
		m.scale.x = m.scale.y = m.scale.z = 0.1
		m.color.r = 0.2
		m.color.g = 0.6
		m.color.b = 1.0
		m.color.a = 0.8
		m.id = 1
		for (x, y, z) in candidates:
			p = PoseStamped().pose
			p.position.x, p.position.y, p.position.z = x, y, z
			m.points.append(p.position)
		arr.markers.append(m)
		# Chosen
		if chosen is not None:
			c = Marker()
			c.header.frame_id = 'world'
			c.type = Marker.SPHERE
			c.action = Marker.ADD
			c.scale.x = c.scale.y = c.scale.z = 0.18
			c.color.r, c.color.g, c.color.b, c.color.a = color_choice[0], color_choice[1], color_choice[2], 0.95
			c.id = 2
			c.pose.position.x, c.pose.position.y, c.pose.position.z = chosen
			arr.markers.append(c)
			# Arrow from signpost to chosen direction
			ar = Marker()
			ar.header.frame_id = 'world'
			ar.type = Marker.ARROW
			ar.action = Marker.ADD
			ar.scale.x = 0.05
			ar.scale.y = 0.1
			ar.scale.z = 0.1
			ar.color.r = 0.0
			ar.color.g = 1.0
			ar.color.b = 0.0
			ar.color.a = 0.9
			ar.id = 3
			# Arrow uses points (start,end)
			ps = PoseStamped().pose.position
			ps.x, ps.y, ps.z = P_sp
			pe = PoseStamped().pose.position
			pe.x, pe.y, pe.z = chosen
			ar.points.append(ps)
			ar.points.append(pe)
			arr.markers.append(ar)
		self.marker_pub.publish(arr)

	def publish_path(self, pts: List[Tuple[float, float, float]]):
		arr = MarkerArray()
		line = Marker()
		line.header.frame_id = 'world'
		line.type = Marker.LINE_STRIP
		line.action = Marker.ADD
		line.scale.x = 0.05
		line.color.r = 0.0
		line.color.g = 0.8
		line.color.b = 0.2
		line.color.a = 0.95
		line.id = 10
		for (x, y, z) in pts:
			p = PoseStamped().pose.position
			p.x, p.y, p.z = x, y, z
			line.points.append(p)
		arr.markers.append(line)
		self.marker_pub.publish(arr)

	def nearest_obstacle_dist(self, qx: float, qy: float, qz: float) -> float:
		pts = self.cloud_points
		if not pts:
			return float('inf')
		best = float('inf')
		for (x, y, z) in pts:
			dx, dy, dz = x - qx, y - qy, z - qz
			d = dx*dx + dy*dy + dz*dz
			if d < best:
				best = d
		return math.sqrt(best)

	def compute_left_direction(self, yaw_deg_opt: Optional[float]) -> Tuple[float, float]:
		# 左侧方向：若有 yaw（墙外法向朝外），取其左手切向（绕+Z 旋转 +90°）
		# 若无 yaw，默认门朝 +X，左侧为 +Y
		if yaw_deg_opt is not None:
			fx, fy = math.cos(math.radians(yaw_deg_opt)), math.sin(math.radians(yaw_deg_opt))
			# 法向 (fx, fy) 朝外，左侧切向 = 旋转 +90°
			lx, ly = -fy, fx
		else:
			lx, ly = 0.0, 1.0
		return normalize2(lx, ly)

	def shape_targets_for_signpost(self, P_sp: Tuple[float, float, float], yaw_deg_opt: Optional[float]) -> Optional[List[Tuple[float, float, float]]]:
		if self.altitude is not None:
			P_sp = (P_sp[0], P_sp[1], float(self.altitude))
		lx, ly = self.compute_left_direction(yaw_deg_opt if self.fixed_left_deg is None else self.fixed_left_deg)
		# 在左侧扇区内采样
		base = (lx, ly)
		angles = [(-self.sector_deg/2.0) + i * (self.sector_deg / max(1, self.sample_count-1)) for i in range(self.sample_count)]
		cands: List[Tuple[float, float, float]] = []
		for ang in angles:
			dx, dy = rot2d(base[0], base[1], ang)
			x = P_sp[0] + dx * self.radius
			y = P_sp[1] + dy * self.radius
			z = P_sp[2]
			cands.append((x, y, z))
		# 打分：距离障碍越大越好
		best_q = None
		best_score = -1e9
		for (x, y, z) in cands:
			d_obs = self.nearest_obstacle_dist(x, y, z)
			score = d_obs
			if d_obs >= self.safe_dist and score > best_score:
				best_score = score
				best_q = (x, y, z)
		self.publish_markers(P_sp, cands, best_q)
		if best_q is None:
			return None
		# pre / pass / post
		vx, vy = normalize2(best_q[0] - P_sp[0], best_q[1] - P_sp[1])
		pre = (best_q[0] - vx * self.backoff_pre, best_q[1] - vy * self.backoff_pre, best_q[2])
		pas = (P_sp[0] + vx * self.delta_in, P_sp[1] + vy * self.delta_in, P_sp[2])
		post = (pas[0] + vx * self.delta_post, pas[1] + vy * self.delta_post, pas[2])
		# 发布折线用于可视化序列
		self.publish_path([pre, pas, post])
		return [pre, pas, post]

	def tick(self):
		if not self.signposts:
			return
		if self.trigger_topic and not self.triggered:
			return
		# 先仅针对第一个 signpost
		wx, wy, wz, yaw_deg = self.signposts[0]
		targets = self.shape_targets_for_signpost((wx, wy, wz), yaw_deg)
		if not targets:
			self.get_logger().warn("No feasible target in left sector; consider increasing --radius or relaxing --safe_dist")
			return
		# 简单发布 pre→pass→post（可被 goal_filter_offset 二次处理）
		for (x, y, z) in targets:
			self.goal_pub.publish(build_pose_stamped(x, y, z))
			self.get_logger().info(f"Published shaped goal: ({x:.2f},{y:.2f},{z:.2f}) -> {self.goal_out}")
		# 单次触发，发布后停止计时器，防止重复刷目标
		self.timer.cancel()


def main():
	parser = argparse.ArgumentParser(description='Shape goals near a door using left-sector sampling around signpost')
	parser.add_argument('--in_cloud', default='/drone_0/cloud')
	parser.add_argument('--odom', default='/drone_0/odom')
	parser.add_argument('--goal_out', default='/move_base_simple/goal')
	parser.add_argument('--trigger', default='')
	parser.add_argument('--json', default='')
	parser.add_argument('--filter', default='Signpost_')
	parser.add_argument('--limit', type=int, default=1)
	parser.add_argument('--sector_deg', type=float, default=80.0)
	parser.add_argument('--radius', type=float, default=1.2)
	parser.add_argument('--safe', dest='safe_dist', type=float, default=0.4)
	parser.add_argument('--delta_in', type=float, default=0.6)
	parser.add_argument('--delta_post', type=float, default=1.2)
	parser.add_argument('--backoff_pre', type=float, default=0.4)
	parser.add_argument('--samples', type=int, default=31)
	parser.add_argument('--stride', type=int, default=2)
	parser.add_argument('--alt', dest='altitude', type=float, default=0.0)
	parser.add_argument('--left_deg', dest='fixed_left_deg', type=float, default=None)
	args = parser.parse_args()

	json_path = args.json if args.json else None
	altitude = args.altitude if (args.altitude and args.altitude > 0.0) else None
	fixed_left_deg = args.fixed_left_deg

	rclpy.init()
	node = DoorGoalShaper(
		cloud_topic=args.in_cloud,
		odom_topic=args.odom,
		goal_out=args.goal_out,
		trigger_topic=args.trigger,
		json_path=json_path,
		name_filter=args.filter,
		limit=args.limit,
		sector_deg=args.sector_deg,
		radius=args.radius,
		safe_dist=args.safe_dist,
		delta_in=args.delta_in,
		delta_post=args.delta_post,
		backoff_pre=args.backoff_pre,
		sample_count=args.samples,
		stride=args.stride,
		altitude=altitude,
		fixed_left_deg=fixed_left_deg,
	)
	try:
		rclpy.spin(node)
	except KeyboardInterrupt:
		pass
	finally:
		node.destroy_node()
		rclpy.shutdown()


if __name__ == '__main__':
	main() 