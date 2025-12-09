# comms.py - simple TCP listener for ambulance coordinates (demo)
import socket
import threading
import json
import logging

logger = logging.getLogger("comms")

def receive_coordinates_from_ambulance(host='127.0.0.1', port=9000, handler=None):
    def server():
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind((host, port))
        s.listen(1)
        logger.info(f"Ambulance listener running on {host}:{port}")
        while True:
            conn, addr = s.accept()
            data = conn.recv(4096)
            conn.close()
            try:
                obj = json.loads(data.decode('utf-8'))
                logger.info(f"Received coords: {obj}")
                if handler:
                    handler(obj)
            except Exception:
                logger.exception("Invalid data received")
    t = threading.Thread(target=server, daemon=True)
    t.start()
    return t

def send_drone_status(url, payload):
    # small wrapper (no requests here to keep simple)
    logger.info(f"[DRY-RUN] send_drone_status to {url} payload {payload}")
    return True

def upload_telemetry(endpoint, telemetry):
    logger.info(f"[DRY-RUN] upload telemetry {telemetry}")
    return True
