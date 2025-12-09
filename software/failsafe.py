# failsafe.py - simple triggers
import logging
logger = logging.getLogger("failsafe")

def failsafe_low_battery(vehicle=None):
    logger.warning("Low battery: would RTL or LAND here")

def failsafe_gps_loss(vehicle=None):
    logger.warning("GPS lost: would LAND here")

def failsafe_comm_loss(vehicle=None):
    logger.warning("Comm loss: would RTL here")
