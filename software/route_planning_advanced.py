# route_planning_advanced.py
# Simple lat/lon grid A* planner with haversine cost and polygon no-fly zones.
import math
from heapq import heappush, heappop
from shapely.geometry import Point, Polygon   # pip install shapely

# convert meters to degrees approx (small distances)
def meters_to_degrees_lat(m):
    return m / 111111.0
def meters_to_degrees_lon(m, lat):
    return m / (111111.0 * math.cos(math.radians(lat)))

def haversine(a, b):
    # a,b = (lat,lon)
    R=6371000.0
    lat1, lon1 = math.radians(a[0]), math.radians(a[1])
    lat2, lon2 = math.radians(b[0]), math.radians(b[1])
    dlat = lat2 - lat1
    dlon = lon2 - lon1
    x = math.sin(dlat/2)**2 + math.cos(lat1)*math.cos(lat2)*math.sin(dlon/2)**2
    return 2*R*math.asin(math.sqrt(x))

def build_grid_bbox(center, radius_m=500, step_m=20):
    # returns list of grid cells centers (lat,lon)
    lat0, lon0 = center
    lat_steps = int((radius_m*2)/step_m) + 1
    lon_steps = lat_steps
    cells = []
    for i in range(lat_steps):
        for j in range(lon_steps):
            dy = (i - lat_steps//2) * step_m
            dx = (j - lon_steps//2) * step_m
            lat = lat0 + meters_to_degrees_lat(dy)
            lon = lon0 + meters_to_degrees_lon(dx, lat0)
            cells.append((lat, lon))
    return cells, lat_steps, lon_steps

def nearest_cell_index(point, cells, lat_steps, lon_steps):
    # return index in flattened grid closest to point
    best = 0
    bestd = float('inf')
    for idx, c in enumerate(cells):
        d = haversine(point, c)
        if d < bestd:
            bestd = d; best = idx
    return best

def idx_to_coords(idx, cells):
    return cells[idx]

def get_neighbors(idx, lat_steps, lon_steps):
    i = idx // lon_steps
    j = idx % lon_steps
    neighbors = []
    for di in (-1,0,1):
        for dj in (-1,0,1):
            if di==0 and dj==0: continue
            ni = i + di
            nj = j + dj
            if 0 <= ni < lat_steps and 0 <= nj < lon_steps:
                neighbors.append(ni*lon_steps + nj)
    return neighbors

def a_star_grid(start, goal, center, radius_m=500, step_m=20, no_fly_polygons=[]):
    cells, lat_steps, lon_steps = build_grid_bbox(center, radius_m, step_m)
    start_idx = nearest_cell_index(start, cells, lat_steps, lon_steps)
    goal_idx = nearest_cell_index(goal, cells, lat_steps, lon_steps)

    # precompute shapely polygons for no-fly zones
    nf_polys = [Polygon(p) for p in no_fly_polygons]

    def walkable(idx):
        # check within no-fly polygons
        lat, lon = cells[idx]
        pt = Point(lon, lat)  # shapely uses lon,lat (x,y)
        for poly in nf_polys:
            if poly.contains(pt):
                return False
        return True

    openheap = []
    heappush(openheap, (0 + haversine(cells[start_idx], cells[goal_idx]), 0, start_idx, None))
    came_from = {}
    gscore = {start_idx: 0}
    while openheap:
        f, g, cur, parent = heappop(openheap)
        if cur in came_from:
            continue
        came_from[cur] = parent
        if cur == goal_idx:
            # build path
            path = []
            node = cur
            while node is not None:
                path.append(cells[node])
                node = came_from[node]
            path.reverse()
            return path
        for nb in get_neighbors(cur, lat_steps, lon_steps):
            if not walkable(nb): continue
            tentative = g + haversine(cells[cur], cells[nb])
            if nb not in gscore or tentative < gscore[nb]:
                gscore[nb] = tentative
                heappush(openheap, (tentative + haversine(cells[nb], cells[goal_idx]), tentative, nb, cur))
    return None
