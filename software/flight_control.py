# flight_control.py - minimal DroneKit wrapper (safe for dry-run)
import time
import logging

logger = logging.getLogger("flight_control")
VEHICLE = None

try:
    from dronekit import connect, VehicleMode, LocationGlobalRelative
except Exception:
    connect = None
    VehicleMode = None
    LocationGlobalRelative = None

def connect_vehicle(connection_string='127.0.0.1:14550', timeout=30):
    global VEHICLE
    if connect is None:
        logger.info("dronekit not available - running in dry-run mode")
        VEHICLE = None
        return None
    logger.info(f"Connecting to vehicle at {connection_string}")
    VEHICLE = connect(connection_string, wait_ready=True, timeout=timeout)
    logger.info("Connected to vehicle")
    return VEHICLE

def arm_and_takeoff(altitude=10):
    if VEHICLE is None:
        logger.info(f"[DRY-RUN] arm_and_takeoff to {altitude}m")
        time.sleep(1)
        return
    logger.info("Arming and taking off")
    VEHICLE.mode = VehicleMode("GUIDED")
    VEHICLE.armed = True
    while not VEHICLE.armed:
        logger.info("Waiting for arming...")
        time.sleep(0.5)
    VEHICLE.simple_takeoff(altitude)
    while True:
        alt = VEHICLE.location.global_relative_frame.alt
        logger.info(f"Altitude: {alt}")
        if alt >= altitude * 0.95:
            break
        time.sleep(0.5)

def goto_location(lat, lon, alt):
    if VEHICLE is None:
        logger.info(f"[DRY-RUN] goto_location: {lat},{lon},{alt}")
        return
    loc = LocationGlobalRelative(lat, lon, alt)
    VEHICLE.simple_goto(loc)

def return_to_launch():
    if VEHICLE is None:
        logger.info("[DRY-RUN] RTL")
        return
    VEHICLE.mode = VehicleMode("RTL")

def land_now():
    if VEHICLE is None:
        logger.info("[DRY-RUN] LAND now")
        return
    VEHICLE.mode = VehicleMode("LAND")

def check_gps_lock(min_sats=6):
    if VEHICLE is None:
        logger.info("[DRY-RUN] pretend GPS OK")
        return True
    sats = VEHICLE.gps_0.satellites_visible
    return sats >= min_sats

def check_battery(min_voltage=10.5):
    if VEHICLE is None:
        logger.info("[DRY-RUN] pretend battery OK")
        return True
    v = VEHICLE.battery.voltage
    return v is not None and v > min_voltage

def failsafe_routine(reason='unknown'):
    logger.warning(f"Failsafe: {reason}")
    try:
        if VEHICLE:
            VEHICLE.mode = VehicleMode("LAND")
    except Exception:
        logger.exception("Failsafe error")
