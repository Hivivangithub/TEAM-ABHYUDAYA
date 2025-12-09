# route_planning.py - simple stubs for path planning (grid demo)
import math

def load_map():
    # small empty grid; replace later with real map
    return {'grid': [[0]*50 for _ in range(50)], 'width':50,'height':50}

def get_distance(a,b):
    return math.hypot(b[0]-a[0], b[1]-a[1])

def a_star(start, goal, mapdata=None):
    # very simple straight-line "path" for demo
    return [start, goal]

def dijkstra(start, goal, mapdata=None):
    return a_star(start, goal, mapdata)

def avoid_no_fly_zones(path, zones=None):
    return path

def optimize_path(path):
    return path

def select_fastest_route(candidates):
    if not candidates:
        return None
    return sorted(candidates, key=lambda x:x[1])[0][0]
