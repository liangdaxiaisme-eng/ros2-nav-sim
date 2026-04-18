"""
复杂地图生成器 — 模拟大型办公/仓库混合环境
支持多种建筑元素：房间、走廊、开放区域、障碍物
"""
import numpy as np
import random
from dataclasses import dataclass, field
from typing import List, Tuple, Optional
from enum import IntEnum


class CellType(IntEnum):
    FREE = 0
    WALL = 1
    FURNITURE = 2
    PILLAR = 3
    DOOR = 4
    SHELF = 5
    PLANT = 6
    VENT = 7  # 通风口/地板纹理


@dataclass
class Room:
    x: int
    y: int
    w: int
    h: int
    name: str = ""
    has_furniture: bool = True
    furniture_density: float = 0.3


@dataclass
class Corridor:
    """走廊/通道"""
    x1: int
    y1: int
    x2: int
    y2: int
    width: int = 3


@dataclass
class Door:
    x: int
    y: int
    horizontal: bool = True  # 门的方向
    size: int = 2


class ComplexMapGenerator:
    """生成复杂仿真地图"""

    def __init__(self, width: int = 200, height: int = 200, seed: int = 42):
        self.width = width
        self.height = height
        self.seed = seed
        random.seed(seed)
        np.random.seed(seed)

        # 主网格
        self.grid = np.zeros((height, width), dtype=np.uint8)
        # 成本地图 (0-255)
        self.costmap = np.zeros((height, width), dtype=np.float64)
        # 区域类型 (用于可视化着色)
        self.zone_map = np.zeros((height, width), dtype=np.uint8)
        # 房间列表
        self.rooms: List[Room] = []
        self.doors: List[Door] = []
        self.spawn_points: List[Tuple[int, int]] = []

    def generate(self) -> dict:
        """生成完整地图"""
        # 1. 外墙
        self._build_outer_walls()

        # 2. 分区布局 — 大型区域
        zones = self._layout_zones()

        # 3. 房间
        self._build_rooms()

        # 4. 走廊连接
        self._build_corridors()

        # 5. 家具和障碍物
        self._place_furniture()

        # 6. 柱子
        self._place_pillars()

        # 7. 植物/装饰
        self._place_decorations()

        # 8. 生成成本地图
        self._generate_costmap()

        # 9. 生成生成点
        self._generate_spawn_points()

        return {
            'grid': self.grid,
            'costmap': self.costmap,
            'zone_map': self.zone_map,
            'rooms': self.rooms,
            'doors': self.doors,
            'spawn_points': self.spawn_points,
            'width': self.width,
            'height': self.height,
            'resolution': 0.05,  # 5cm per cell
        }

    def _build_outer_walls(self):
        """外墙"""
        wall_thickness = 3
        self.grid[:wall_thickness, :] = CellType.WALL
        self.grid[-wall_thickness:, :] = CellType.WALL
        self.grid[:, :wall_thickness] = CellType.WALL
        self.grid[:, -wall_thickness:] = CellType.WALL

    def _layout_zones(self) -> List[Tuple[int, int, int, int]]:
        """将地图分为多个区域"""
        zones = []
        # 左上: 大会议室
        zones.append((5, 5, 70, 60))
        # 右上: 开放办公区
        zones.append((75, 5, 120, 60))
        # 左中: 仓库/储藏区
        zones.append((5, 65, 70, 130))
        # 右中: 实验室
        zones.append((75, 65, 120, 130))
        # 下方: 大厅/走廊区
        zones.append((5, 135, 190, 190))

        for i, (x1, y1, x2, y2) in enumerate(zones):
            self.zone_map[y1:y2, x1:x2] = i + 1
            # 区域边界加粗线
            self.grid[y1, x1:x2] = CellType.WALL
            self.grid[y2 - 1, x1:x2] = CellType.WALL
            self.grid[y1:y2, x1] = CellType.WALL
            self.grid[y1:y2, x2 - 1] = CellType.WALL

        return zones

    def _build_rooms(self):
        """在各区域内生成房间"""
        # 大会议室 — 中央长桌 + 椅子
        self.rooms.append(Room(10, 10, 55, 45, "Conference Room", True, 0.2))

        # 小办公室
        for i in range(4):
            x = 80 + (i % 2) * 55
            y = 10 + (i // 2) * 22
            self.rooms.append(Room(x, y, 50, 18, f"Office_{i}", True, 0.35))

        # 仓库 — 货架
        self.rooms.append(Room(10, 70, 60, 55, "Warehouse", True, 0.5))

        # 实验室 — 设备
        for i in range(2):
            for j in range(2):
                x = 80 + i * 55
                y = 70 + j * 25
                self.rooms.append(Room(x, y, 50, 22, f"Lab_{i}_{j}", True, 0.25))

        for room in self.rooms:
            self._build_room_walls(room)

    def _build_room_walls(self, room: Room):
        """构建单个房间的墙"""
        x1, y1 = room.x, room.y
        x2, y2 = x1 + room.w, y1 + room.h

        # 确保在地图范围内
        x2 = min(x2, self.width - 4)
        y2 = min(y2, self.height - 4)

        # 四面墙
        self.grid[y1, x1:x2] = CellType.WALL
        self.grid[y2 - 1, x1:x2] = CellType.WALL
        self.grid[y1:y2, x1] = CellType.WALL
        self.grid[y1:y2, x2 - 1] = CellType.WALL

        # 在随机位置开1-2个门
        num_doors = random.randint(1, 2)
        for _ in range(num_doors):
            side = random.randint(0, 3)
            if side == 0:  # 上
                dx = random.randint(x1 + 3, x2 - 5)
                self.grid[y1, dx:dx + 3] = CellType.DOOR
                self.doors.append(Door(dx, y1, True, 3))
            elif side == 1:  # 下
                dx = random.randint(x1 + 3, x2 - 5)
                self.grid[y2 - 1, dx:dx + 3] = CellType.DOOR
                self.doors.append(Door(dx, y2 - 1, True, 3))
            elif side == 2:  # 左
                dy = random.randint(y1 + 3, y2 - 5)
                self.grid[dy:dy + 3, x1] = CellType.DOOR
                self.doors.append(Door(x1, dy, False, 3))
            else:  # 右
                dy = random.randint(y1 + 3, y2 - 5)
                self.grid[dy:dy + 3, x2 - 1] = CellType.DOOR
                self.doors.append(Door(x2 - 1, dy, False, 3))

    def _build_corridors(self):
        """构建连接走廊"""
        corridor_configs = [
            # 主水平走廊
            (5, 62, 190, 65),
            (5, 132, 190, 135),
            # 主垂直走廊
            (72, 5, 75, 190),
            (122, 5, 125, 190),
            # 对角走廊
            (30, 95, 72, 98),
            (125, 85, 165, 88),
        ]

        for x1, y1, x2, y2 in corridor_configs:
            x2 = min(x2, self.width - 4)
            y2 = min(y2, self.height - 4)
            self.grid[y1:y2, x1:x2] = CellType.FREE
            # 走廊边界
            self.grid[y1, x1:x2] = CellType.WALL
            self.grid[y2 - 1, x1:x2] = CellType.WALL
            self.grid[y1:y2, x1] = CellType.WALL
            self.grid[y1:y2, x2 - 1] = CellType.WALL
            # 走廊内部设为 FREE
            self.grid[y1 + 1:y2 - 1, x1 + 1:x2 - 1] = CellType.FREE
            # 走廊 zone
            self.zone_map[y1 + 1:y2 - 1, x1 + 1:x2 - 1] = 6  # corridor zone

    def _place_furniture(self):
        """在房间内放置家具"""
        for room in self.rooms:
            if not room.has_furniture:
                continue

            inner_x1 = room.x + 2
            inner_y1 = room.y + 2
            inner_x2 = room.x + room.w - 2
            inner_y2 = room.y + room.h - 2

            if "Warehouse" in room.name:
                self._place_shelves(inner_x1, inner_y1, inner_x2, inner_y2)
            elif "Conference" in room.name:
                self._place_conference_table(inner_x1, inner_y1, inner_x2, inner_y2)
            elif "Office" in room.name:
                self._place_desks(inner_x1, inner_y1, inner_x2, inner_y2)
            elif "Lab" in room.name:
                self._place_lab_equipment(inner_x1, inner_y1, inner_x2, inner_y2)

    def _place_shelves(self, x1, y1, x2, y2):
        """仓库货架 — 平行排列"""
        shelf_width = 2
        spacing = 8
        for sx in range(x1 + 2, x2 - 2, spacing):
            self.grid[y1 + 2:y2 - 2, sx:sx + shelf_width] = CellType.SHELF

    def _place_conference_table(self, x1, y1, x2, y2):
        """会议桌"""
        cx = (x1 + x2) // 2
        cy = (y1 + y2) // 2
        # 椭圆形长桌
        for row in range(cy - 10, cy + 10):
            for col in range(cx - 15, cx + 15):
                if row < y1 or row >= y2 or col < x1 or col >= x2:
                    continue
                # 椭圆判断
                ry = (row - cy) / 10
                rx = (col - cx) / 15
                if rx ** 2 + ry ** 2 < 1:
                    self.grid[row, col] = CellType.FURNITURE

        # 椅子 (小方块围绕桌子)
        for angle_offset in range(0, 360, 30):
            import math
            rad = math.radians(angle_offset)
            chair_x = int(cx + 18 * math.cos(rad))
            chair_y = int(cy + 13 * math.sin(rad))
            if y1 < chair_y < y2 and x1 < chair_x < x2:
                self.grid[chair_y, chair_x] = CellType.FURNITURE
                self.grid[chair_y, chair_x + 1] = CellType.FURNITURE

    def _place_desks(self, x1, y1, x2, y2):
        """办公桌 — 2x2 网格"""
        desks_per_row = max(1, (x2 - x1) // 12)
        desks_per_col = max(1, (y2 - y1) // 10)
        for dy in range(desks_per_col):
            for dx in range(desks_per_row):
                desk_x = x1 + 3 + dx * 12
                desk_y = y1 + 3 + dy * 10
                if desk_x + 5 < x2 and desk_y + 3 < y2:
                    self.grid[desk_y:desk_y + 3, desk_x:desk_x + 5] = CellType.FURNITURE
                    # 椅子
                    if desk_y - 1 > y1:
                        self.grid[desk_y - 1, desk_x + 2] = CellType.FURNITURE

    def _place_lab_equipment(self, x1, y1, x2, y2):
        """实验室设备 — 随机散布"""
        num_items = random.randint(4, 8)
        for _ in range(num_items):
            ix = random.randint(x1 + 2, x2 - 5)
            iy = random.randint(y1 + 2, y2 - 5)
            iw = random.randint(2, 4)
            ih = random.randint(2, 3)
            if ix + iw < x2 and iy + ih < y2:
                self.grid[iy:iy + ih, ix:ix + iw] = CellType.FURNITURE

    def _place_pillars(self):
        """在走廊和大厅放置支撑柱"""
        # 大厅区域柱子
        for px in range(20, 180, 15):
            for py in range(140, 185, 15):
                if self.grid[py, px] == CellType.FREE:
                    self.grid[py, px] = CellType.PILLAR
                    self.grid[py, px + 1] = CellType.PILLAR
                    self.grid[py + 1, px] = CellType.PILLAR
                    self.grid[py + 1, px + 1] = CellType.PILLAR

    def _place_decorations(self):
        """装饰物 — 植物、通风口等"""
        # 通风口 (地板纹理标记)
        for _ in range(20):
            vx = random.randint(10, self.width - 10)
            vy = random.randint(10, self.height - 10)
            if self.grid[vy, vx] == CellType.FREE:
                self.grid[vy, vx] = CellType.VENT

        # 盆栽
        for _ in range(15):
            px = random.randint(5, self.width - 5)
            py = random.randint(5, self.height - 5)
            if self.grid[py, px] == CellType.FREE and self.zone_map[py, px] == 6:
                self.grid[py, px] = CellType.PLANT

    def _generate_costmap(self):
        """生成代价地图 (含膨胀层)"""
        from scipy.ndimage import distance_transform_edt

        # 障碍物 = WALL + FURNITURE + PILLAR + SHELF
        obstacle_mask = np.isin(self.grid, [
            CellType.WALL, CellType.FURNITURE, CellType.PILLAR, CellType.SHELF
        ])

        # 距离变换
        free_space = ~obstacle_mask
        if free_space.any():
            distances = distance_transform_edt(free_space)
            # 膨胀层: 5 cells 半径内按距离衰减
            inflation_radius = 5.0
            self.costmap = np.where(
                distances <= inflation_radius,
                255 * np.exp(-distances / (inflation_radius / 3)),
                0.0
            )
        else:
            self.costmap = np.zeros_like(self.grid, dtype=np.float64)

    def _generate_spawn_points(self):
        """生成有效生成点"""
        free_cells = np.argwhere(self.grid == CellType.FREE)
        if len(free_cells) == 0:
            return

        # 在各区域中心附近找生成点
        for zone_id in range(1, 7):
            zone_cells = np.argwhere((self.grid == CellType.FREE) & (self.zone_map == zone_id))
            if len(zone_cells) > 50:
                # 取区域中心附近的点
                cy, cx = zone_cells.mean(axis=0).astype(int)
                # 找最近的 free cell
                dists = np.sqrt(((zone_cells - [cy, cx]) ** 2).sum(axis=1))
                best_idx = dists.argsort()[:3]
                for idx in best_idx:
                    sy, sx = zone_cells[idx]
                    self.spawn_points.append((int(sx), int(sy)))


def generate_complex_map(seed=42):
    """便捷函数"""
    gen = ComplexMapGenerator(width=200, height=200, seed=seed)
    return gen.generate()


if __name__ == "__main__":
    import time
    print("Generating complex map...")
    t0 = time.time()
    data = generate_complex_map()
    elapsed = time.time() - t0

    print(f"Map: {data['width']}x{data['height']} ({data['resolution']*100}cm/cell)")
    print(f"Rooms: {len(data['rooms'])}")
    print(f"Doors: {len(data['doors'])}")
    print(f"Spawn points: {len(data['spawn_points'])}")
    print(f"Walls: {(data['grid'] == 1).sum()}")
    print(f"Furniture: {(data['grid'] == 2).sum()}")
    print(f"Generated in {elapsed:.2f}s")
