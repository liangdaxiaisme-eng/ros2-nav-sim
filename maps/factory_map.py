"""
工厂地图生成器 — 模拟真实工业巡检环境
大型厂房 (40m x 30m), AGV 巡检路线
"""
import numpy as np
import random
import math
from dataclasses import dataclass, field
from typing import List, Tuple


class Cell:
    FREE = 0
    WALL = 1          # 墙壁/边界
    PILLAR = 2        # 钢结构柱
    CONVEYOR = 3      # 传送带
    MACHINE = 4       # 重型机械
    SHELF = 5         # 货架/料架
    PIPE = 6          # 管道
    VENT = 7          # 通风口/地沟
    MARK = 8          # 地面标线
    LOADING = 9       # 装卸区
    CHARGER = 10      # AGV充电站
    SAFETY = 11       # 安全围栏


class FactoryMapGenerator:
    """生成真实工厂地图"""

    def __init__(self, width=800, height=600, resolution=0.05):
        self.W = width
        self.H = height
        self.res = resolution  # meters per cell
        self.grid = np.zeros((height, width), dtype=np.uint8)
        self.costmap = np.zeros((height, width), dtype=np.float64)
        self.zone_map = np.zeros((height, width), dtype=np.uint8)
        self.spawn_points: List[Tuple[int, int]] = []
        self.waypoints: List[Tuple[int, int]] = []

    def generate(self) -> dict:
        self._outer_walls(3)

        # ── 区域划分 ──
        #  A区: 原材料仓库 (左上)
        self._raw_material_warehouse(10, 10, 250, 200)
        #  B区: 生产线1 (中上)
        self._production_line(260, 10, 540, 130, line_id=1)
        #  C区: 生产线2 (中上偏下)
        self._production_line(260, 140, 540, 260, line_id=2)
        #  D区: 质检区 (右上)
        self._quality_inspection(550, 10, 790, 130)
        #  E区: 成品仓库 (右中)
        self._finished_goods_warehouse(550, 140, 790, 350)
        #  F区: 维修/工具间 (左中)
        self._maintenance_room(10, 210, 250, 350)
        #  G区: 中央通道 (横向主干道)
        self._main_corridor_h(10, 355, 790, 385)
        #  H区: 装卸码头 (下方)
        self._loading_dock(10, 390, 790, 590)

        # ── 连通走廊 ──
        self._corridors()

        # ── 柱子 ──
        self._structural_pillars()

        # ── 地面标线 ──
        self._floor_markings()

        # ── AGV充电站 ──
        self._charging_stations()

        # ── 生成代价地图 ──
        self._build_costmap()

        # ── 生成巡检航点 ──
        self._build_inspection_waypoints()

        # ── 生成点 ──
        self._build_spawn_points()

        return {
            'grid': self.grid,
            'costmap': self.costmap,
            'zone_map': self.zone_map,
            'width': self.W,
            'height': self.H,
            'resolution': self.res,
            'spawn_points': self.spawn_points,
            'inspection_waypoints': self.waypoints,
            'map_size_m': (self.W * self.res, self.H * self.res),
        }

    # ══════════════════════════════════════════
    #  区域构建
    # ══════════════════════════════════════════

    def _outer_walls(self, t):
        self.grid[:t, :] = Cell.WALL
        self.grid[-t:, :] = Cell.WALL
        self.grid[:, :t] = Cell.WALL
        self.grid[:, -t:] = Cell.WALL

    def _box(self, x1, y1, x2, y2, wall=True, zone_id=0):
        """画区域边界框"""
        x2 = min(x2, self.W - 4)
        y2 = min(y2, self.H - 4)
        if wall:
            self.grid[y1, x1:x2] = Cell.WALL
            self.grid[y2 - 1, x1:x2] = Cell.WALL
            self.grid[y1:y2, x1] = Cell.WALL
            self.grid[y1:y2, x2 - 1] = Cell.WALL
        if zone_id:
            self.zone_map[y1 + 1:y2 - 1, x1 + 1:x2 - 1] = zone_id

    def _door_h(self, x, y, size=4):
        """水平方向门洞"""
        self.grid[y, x:x + size] = Cell.FREE

    def _door_v(self, x, y, size=4):
        """垂直方向门洞"""
        self.grid[y:y + size, x] = Cell.FREE

    def _raw_material_warehouse(self, x1, y1, x2, y2):
        """A区 — 原材料仓库：货架+叉车通道"""
        self._box(x1, y1, x2, y2, zone_id=1)
        # 货架 — 纵向排列，间隔 8 格 (叉车通道)
        shelf_len = y2 - y1 - 20
        for sx in range(x1 + 15, x2 - 10, 18):
            # 货架主体
            self.grid[y1 + 8:y1 + 8 + shelf_len, sx:sx + 3] = Cell.SHELF
            # 货架端部标记
            self.grid[y1 + 8, sx:sx + 3] = Cell.SAFETY
            self.grid[y1 + 8 + shelf_len - 1, sx:sx + 3] = Cell.SAFETY

        # 门 (2个)
        self._door_h(x2 - 1, y1 + 50, 3)
        self._door_h(x2 - 1, y1 + 120, 3)

    def _production_line(self, x1, y1, x2, y2, line_id=1):
        """B/C区 — 生产线：传送带+设备"""
        self._box(x1, y1, x2, y2, zone_id=2 + line_id - 1)

        # 中央传送带 (贯穿)
        conv_y = (y1 + y2) // 2
        self.grid[conv_y - 1:conv_y + 1, x1 + 3:x2 - 3] = Cell.CONVEYOR

        # 两侧设备
        for mx in range(x1 + 20, x2 - 15, 35):
            # 上方设备
            self.grid[y1 + 8:y1 + 20, mx:mx + 8] = Cell.MACHINE
            # 下方设备
            self.grid[y2 - 20:y2 - 8, mx:mx + 8] = Cell.MACHINE
            # 设备旁边的管道
            self.grid[y1 + 20:y1 + 22, mx:mx + 8] = Cell.PIPE
            self.grid[y2 - 22:y2 - 20, mx:mx + 8] = Cell.PIPE

        # 两端门
        self._door_v(x1, conv_y - 3, 6)
        self._door_v(x2 - 1, conv_y - 3, 6)

    def _quality_inspection(self, x1, y1, x2, y2):
        """D区 — 质检区：检测台+仪器"""
        self._box(x1, y1, x2, y2, zone_id=4)

        # 检测台 (网格排列)
        for iy in range(y1 + 15, y2 - 10, 25):
            for ix in range(x1 + 15, x2 - 15, 30):
                self.grid[iy:iy + 4, ix:ix + 10] = Cell.MACHINE
                # 台前空间 (操作区)
                self.grid[iy + 5:iy + 8, ix:ix + 10] = Cell.FREE

        # 门
        self._door_h(x1, y1 + 50, 3)
        self._door_v(x1, (y1 + y2) // 2 - 2, 5)

    def _finished_goods_warehouse(self, x1, y1, x2, y2):
        """E区 — 成品仓库"""
        self._box(x1, y1, x2, y2, zone_id=5)

        # 大型货架
        for sy in range(y1 + 10, y2 - 10, 20):
            self.grid[sy, x1 + 10:x2 - 10] = Cell.SHELF
            self.grid[sy + 1, x1 + 10:x2 - 10] = Cell.SHELF

        # 门
        self._door_h(x1, y1 + 50, 3)
        self._door_h(x1, (y1 + y2) // 2, 3)

    def _maintenance_room(self, x1, y1, x2, y2):
        """F区 — 维修/工具间"""
        self._box(x1, y1, x2, y2, zone_id=6)

        # 工具柜
        for tx in range(x1 + 10, x2 - 10, 20):
            self.grid[y1 + 5:y1 + 10, tx:tx + 5] = Cell.SHELF
            self.grid[y2 - 10:y2 - 5, tx:tx + 5] = Cell.SHELF

        # 工作台
        cy = (y1 + y2) // 2
        self.grid[cy - 3:cy + 3, x1 + 20:x1 + 60] = Cell.MACHINE

        # 门
        self._door_h(x2 - 1, cy - 2, 5)

    def _main_corridor_h(self, x1, y1, x2, y2):
        """G区 — 横向主干道 (AGV 专用)"""
        self._box(x1, y1, x2, y2, zone_id=7)
        # 内部清空
        self.grid[y1 + 1:y2 - 1, x1 + 1:x2 - 1] = Cell.FREE
        # 中心标线
        cy = (y1 + y2) // 2
        for mx in range(x1 + 5, x2 - 5, 12):
            self.grid[cy, mx:mx + 6] = Cell.MARK

    def _loading_dock(self, x1, y1, x2, y2):
        """H区 — 装卸码头"""
        self._box(x1, y1, x2, y2, zone_id=8)

        # 装卸位 (底部开口)
        dock_positions = []
        for dx in range(x1 + 30, x2 - 30, 50):
            # 装卸平台
            self.grid[y2 - 25:y2 - 5, dx:dx + 20] = Cell.LOADING
            dock_positions.append(dx + 10)
            # 底部开门 (通向外面)
            self.grid[y2 - 1, dx + 5:dx + 15] = Cell.FREE

        # 中间通道
        self.grid[y1 + 1:y2 - 1, (x1 + x2) // 2 - 2:(x1 + x2) // 2 + 2] = Cell.FREE

        # 叉车停放区
        for px in range(x1 + 15, x2 - 15, 40):
            self.grid[y1 + 8:y1 + 12, px:px + 3] = Cell.MARK

    def _corridors(self):
        """连接走廊"""
        # 纵向通道
        for cx in [255, 545]:
            self.grid[10:355, cx:cx + 5] = Cell.FREE
            self.grid[10, cx:cx + 5] = Cell.WALL
            self.grid[354, cx:cx + 5] = Cell.WALL

        # 横向短走廊
        cy_positions = [50, 100, 180, 220]
        for cy in cy_positions:
            if cy < 350:
                # 左侧到中部
                self.grid[cy:cy + 4, 250:260] = Cell.FREE
                # 中部到右侧
                self.grid[cy:cy + 4, 540:550] = Cell.FREE

    def _structural_pillars(self):
        """钢结构支撑柱"""
        for px in range(40, self.W - 30, 50):
            for py in range(40, self.H - 30, 50):
                if self.grid[py, px] == Cell.FREE:
                    self.grid[py:py + 3, px:px + 3] = Cell.PILLAR

    def _floor_markings(self):
        """地面标线 (AGV引导线)"""
        # 主干道引导线
        cy = 370  # 主干道中央
        for mx in range(15, self.W - 15, 8):
            if self.grid[cy, mx] == Cell.FREE:
                self.grid[cy, mx] = Cell.MARK

        # 区域间连接标线
        mark_paths = [
            # A→走廊
            [(125, 200), (125, 360)],
            # B→走廊
            [(400, 130), (400, 355)],
            # 走廊→H
            [(400, 385), (400, 400)],
        ]
        for path in mark_paths:
            (x1, y1), (x2, y2) = path
            steps = max(abs(x2 - x1), abs(y2 - y1))
            for s in range(steps):
                t = s / max(steps, 1)
                mx = int(x1 + t * (x2 - x1))
                my = int(y1 + t * (y2 - y1))
                if 0 < mx < self.W - 1 and 0 < my < self.H - 1:
                    if self.grid[my, mx] == Cell.FREE:
                        self.grid[my, mx] = Cell.MARK

    def _charging_stations(self):
        """AGV充电站"""
        stations = [(30, 365), (400, 365), (750, 365)]
        for sx, sy in stations:
            if self.grid[sy, sx] == Cell.FREE:
                self.grid[sy:sy + 3, sx:sx + 6] = Cell.CHARGER

    def _build_costmap(self):
        """代价地图"""
        from scipy.ndimage import distance_transform_edt
        obstacle = np.isin(self.grid, [Cell.WALL, Cell.PILLAR, Cell.CONVEYOR,
                                        Cell.MACHINE, Cell.SHELF, Cell.PIPE, Cell.SAFETY])
        free = ~obstacle
        if free.any():
            dist = distance_transform_edt(free)
            inflate_r = 4.0
            self.costmap = np.where(dist <= inflate_r,
                                     255 * np.exp(-dist / (inflate_r / 3)), 0.0)

    def _build_inspection_waypoints(self):
        """AGV 巡检航点"""
        r = self.res
        self.waypoints = [
            # 1. 充电站出发
            (400, 370),
            # 2. 原材料仓库巡检
            (60, 100),
            (60, 160),
            (120, 160),
            (120, 100),
            # 3. 生产线1巡检
            (300, 50),
            (400, 50),
            (500, 50),
            # 4. 生产线2巡检
            (500, 200),
            (400, 200),
            (300, 200),
            # 5. 质检区
            (650, 50),
            (650, 100),
            # 6. 成品仓库
            (650, 200),
            (650, 280),
            # 7. 维修间
            (80, 270),
            # 8. 返回充电
            (400, 370),
        ]

    def _build_spawn_points(self):
        """AGV起始位置"""
        self.spawn_points = [(400, 370)]


def generate_factory_map(seed=42):
    np.random.seed(seed)
    random.seed(seed)
    gen = FactoryMapGenerator(width=800, height=600, resolution=0.05)
    return gen.generate()


if __name__ == "__main__":
    import time
    t0 = time.time()
    data = generate_factory_map()
    t1 = time.time()
    w, h = data['map_size_m']
    print(f"Factory map: {data['width']}x{data['height']} cells")
    print(f"Real size: {w:.1f}m x {h:.1f}m ({w*h:.0f} m²)")
    print(f"Walls: {(data['grid']==1).sum()}")
    print(f"Pillars: {(data['grid']==2).sum()}")
    print(f"Conveyors: {(data['grid']==3).sum()}")
    print(f"Machines: {(data['grid']==4).sum()}")
    print(f"Shelves: {(data['grid']==5).sum()}")
    print(f"Inspection waypoints: {len(data['inspection_waypoints'])}")
    print(f"Generated in {t1-t0:.2f}s")
