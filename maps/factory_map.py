"""
工厂地图 — 紧凑型 (20m x 15m)
一条生产线 + 仓库 + 质检 + 充电站，AGV 巡检
"""
import numpy as np
import random
from scipy.ndimage import distance_transform_edt


class Cell:
    FREE = 0
    WALL = 1
    PILLAR = 2
    CONVEYOR = 3
    MACHINE = 4
    SHELF = 5
    PIPE = 6
    MARK = 8
    LOADING = 9
    CHARGER = 10
    SAFETY = 11


def generate_factory_map(seed=42):
    np.random.seed(seed)
    random.seed(seed)

    W, H = 400, 300  # 20m x 15m
    res = 0.05
    grid = np.zeros((H, W), dtype=np.uint8)
    zone_map = np.zeros((H, W), dtype=np.uint8)

    # 外墙
    grid[:3, :] = Cell.WALL
    grid[-3:, :] = Cell.WALL
    grid[:, :3] = Cell.WALL
    grid[:, -3:] = Cell.WALL

    def box(x1, y1, x2, y2, zone=0):
        nonlocal grid, zone_map
        x2, y2 = min(x2, W-4), min(y2, H-4)
        grid[y1, x1:x2] = Cell.WALL
        grid[y2-1, x1:x2] = Cell.WALL
        grid[y1:y2, x1] = Cell.WALL
        grid[y1:y2, x2-1] = Cell.WALL
        if zone:
            zone_map[y1+1:y2-1, x1+1:x2-1] = zone

    def door_h(x, y, sz=4):
        grid[y, x:x+sz] = Cell.FREE

    def door_v(x, y, sz=4):
        grid[y:y+sz, x] = Cell.FREE

    # ════ A区: 原材料仓库 (左上 180x120) ════
    box(10, 10, 180, 120, zone=1)
    # 货架纵向排列
    for sx in range(20, 170, 20):
        grid[20:110, sx:sx+2] = Cell.SHELF
    door_h(179, 55, 3)   # 右侧门通向走廊
    door_h(179, 90, 3)

    # ════ B区: 生产线 (中上 200x100) ════
    box(185, 10, 385, 110, zone=2)
    # 传送带 (中央)
    cy_conv = 60
    grid[cy_conv-1:cy_conv+1, 190:380] = Cell.CONVEYOR
    # 两侧设备
    for mx in range(200, 375, 30):
        grid[20:30, mx:mx+8] = Cell.MACHINE   # 上方
        grid[90:100, mx:mx+8] = Cell.MACHINE  # 下方
        grid[30:32, mx:mx+8] = Cell.PIPE
        grid[88:90, mx:mx+8] = Cell.PIPE
    door_v(185, 56, 5)   # 左门
    door_v(384, 56, 5)   # 右门通质检

    # ════ C区: 质检区 (右上 100x100) ════
    box(390, 10, 490, 110, zone=3)  # 注意: 实际地图宽度400, 但用相对位置
    # 这里需要调整——地图只有400宽，重新布局
    # 改为: 质检区在生产线下方

    # 重新设计布局 (400x300):
    # ┌──────────┬───────────────┐
    # │ A 原料仓  │  B 生产线     │
    # │ 170x110  │  200x110      │
    # ├──────────┼───────────────┤
    # │ D 维修间  │  C 质检区     │
    # │ 170x80   │  200x80       │
    # ├──────────┴───────────────┤
    # │     E 主干道 (AGV专用)    │
    # │       380x30             │
    # ├──────────────────────────┤
    # │     F 装卸/充电区         │
    # │       380x70             │
    # └──────────────────────────┘

    # 重新生成
    grid = np.zeros((H, W), dtype=np.uint8)
    zone_map = np.zeros((H, W), dtype=np.uint8)
    grid[:3, :] = Cell.WALL
    grid[-3:, :] = Cell.WALL
    grid[:, :3] = Cell.WALL
    grid[:, -3:] = Cell.WALL

    # A区: 原料仓库 (左上)
    box(10, 10, 175, 115, zone=1)
    for sx in range(20, 165, 18):
        grid[20:105, sx:sx+2] = Cell.SHELF
    door_h(174, 50, 3)
    door_h(174, 85, 3)

    # B区: 生产线 (右上)
    box(180, 10, 390, 115, zone=2)
    cy = 62
    grid[cy-1:cy+1, 185:385] = Cell.CONVEYOR
    for mx in range(195, 380, 28):
        grid[20:32, mx:mx+7] = Cell.MACHINE
        grid[100:112, mx:mx+7] = Cell.MACHINE
        grid[32:34, mx:mx+7] = Cell.PIPE
        grid[98:100, mx:mx+7] = Cell.PIPE
    door_v(180, 58, 5)
    door_v(389, 58, 5)

    # C区: 质检区 (右中)
    box(180, 120, 390, 195, zone=3)
    for iy in range(130, 185, 22):
        for ix in range(190, 380, 35):
            grid[iy:iy+4, ix:ix+10] = Cell.MACHINE
    door_v(180, 155, 4)
    door_h(280, 194, 4)

    # D区: 维修间 (左中)
    box(10, 120, 175, 195, zone=4)
    # 工具柜
    for tx in range(20, 165, 25):
        grid[125:132, tx:tx+6] = Cell.SHELF
        grid[185:192, tx:tx+6] = Cell.SHELF
    # 工作台
    grid[155:162, 40:100] = Cell.MACHINE
    door_h(174, 155, 4)

    # E区: 主干道 (AGV专用)
    box(10, 200, 390, 225, zone=5)
    grid[201:224, 11:389] = Cell.FREE
    # 标线
    cmid = 212
    for mx in range(15, 385, 10):
        grid[cmid, mx:mx+5] = Cell.MARK
    # 门
    door_v(85, 199, 3)   # 上方连接D
    door_v(280, 199, 3)  # 上方连接C

    # F区: 装卸+充电区 (下方)
    box(10, 230, 390, 292, zone=6)
    # 充电站 (左侧)
    grid[240:246, 25:45] = Cell.CHARGER
    grid[240:246, 55:75] = Cell.CHARGER
    # 装卸位 (右侧)
    grid[250:270, 340:370] = Cell.LOADING
    grid[291, 350:360] = Cell.FREE  # 底部开口
    # 叉车停放标线
    for px in range(100, 330, 30):
        grid[270:273, px:px+2] = Cell.MARK
    door_h(85, 229, 4)
    door_h(280, 229, 4)

    # 结构柱
    for px in range(50, 390, 60):
        for py in range(30, 290, 50):
            if py > 200 and py < 225:
                continue  # 主干道无柱
            if grid[py, px] == Cell.FREE:
                grid[py:py+3, px:px+3] = Cell.PILLAR

    # ── 代价地图 ──
    obstacle = np.isin(grid, [Cell.WALL, Cell.PILLAR, Cell.CONVEYOR,
                               Cell.MACHINE, Cell.SHELF, Cell.PIPE, Cell.SAFETY])
    free = ~obstacle
    dist = distance_transform_edt(free)
    inflate_r = 3.0
    costmap = np.where(dist <= inflate_r, 255 * np.exp(-dist / (inflate_r/3)), 0.0)

    # ── 巡检航点 (格子坐标) ──
    waypoints = [
        (50, 240),    # 充电站出发
        # 原料仓库
        (50, 50), (50, 90), (100, 90), (100, 50),
        # 生产线
        (220, 40), (280, 40), (340, 40),
        (340, 90), (280, 90), (220, 90),
        # 质检区
        (220, 150), (280, 150), (340, 150),
        # 维修间
        (50, 155), (100, 155),
        # 返回
        (50, 240),
    ]

    spawn_points = [(50, 240)]

    return {
        'grid': grid,
        'costmap': costmap,
        'zone_map': zone_map,
        'width': W,
        'height': H,
        'resolution': res,
        'spawn_points': spawn_points,
        'inspection_waypoints': waypoints,
        'map_size_m': (W * res, H * res),
    }


if __name__ == "__main__":
    import time
    t0 = time.time()
    data = generate_factory_map()
    t1 = time.time()
    w, h = data['map_size_m']
    g = data['grid']
    print(f"Factory: {data['width']}x{data['height']} = {w:.0f}m x {h:.0f}m ({w*h:.0f} m²)")
    print(f"Walls: {(g==1).sum()}  Pillars: {(g==2).sum()}  Conveyors: {(g==3).sum()}")
    print(f"Machines: {(g==4).sum()}  Shelves: {(g==5).sum()}  Chargers: {(g==10).sum()}")
    print(f"Waypoints: {len(data['inspection_waypoints'])}")
    print(f"Time: {t1-t0:.2f}s")
