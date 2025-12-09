# obstacle_detection.py - small OpenCV fallback
import cv2
import numpy as np

def detect_objects(frame):
    # fallback: detect large contours, return list of boxes
    try:
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5,5), 0)
        _, th = cv2.threshold(blur, 80, 255, cv2.THRESH_BINARY)
        contours, _ = cv2.findContours(th, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        dets = []
        for c in contours:
            area = cv2.contourArea(c)
            if area < 500: continue
            x,y,w,h = cv2.boundingRect(c)
            dets.append(("object", 0.5, (x,y,w,h)))
        return dets
    except Exception:
        return []

def detect_traffic_density(frame):
    dets = detect_objects(frame)
    count = len(dets)
    if count <= 2: return count, 'low'
    if count <= 6: return count, 'medium'
    return count, 'high'

def reroute_if_traffic_jam(current_path, frame):
    _, density = detect_traffic_density(frame)
    return density == 'high'
