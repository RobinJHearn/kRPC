import logging
import math
import time

import krpc

import krpclogstream
import pid
from vector import Vector

# Configurable Features
FEATURES = {
    "FULL_BURN": False,  # Don't reduce throttle on ascent burn
    "USE_SAS": True,  # Use SAS to point for maneuver
    "PERFORM_ASCENT_ROLL": False,  # Roll rocket on launch ala NASA
    "SHOW_LOG_IN_KSP": False,  # Show log in a window in KSP
}

# Launch parameters
TARGET_ALTITUDE = 100000
TARGET_HEADING = 90

# Universal Constants
G0 = 9.80665

# Configure connection to KSP and some short cuts
conn = krpc.connect(name="Launch into orbit")
ksc = conn.space_center  # pylint: disable=no-member
vessel = ksc.active_vessel
SASMode = ksc.SASMode

# Configure logging
ksp_stream = krpclogstream.KrpcLogStream(conn)
logging.basicConfig(format="%(asctime)-15s %(levelname)s %(message)s")
handler = logging.StreamHandler(ksp_stream)
logger = logging.getLogger(__name__)
logger.addHandler(handler)
logger.setLevel(logging.INFO)


def show_features():
    """Log out the configurable features and their state"""
    for k, v in FEATURES.items():
        logger.info(f"{k} -> {v}")


def limit(value, min_value=0, max_value=1):
    """Limit a `value` to between the `minValue` and `maxValue` inclusive

    Parameters:
    `value`: value to limit
    `min_value`: minimum value allowed
    `max_value: maximum value allowed
    """
    return min(max(value, min_value), max_value)


def auto_stage():
    """Stage engines when thrust drops"""
    if not hasattr(auto_stage, "available_thrust"):
        auto_stage.available_thrust = vessel.available_thrust - 10
    if vessel.available_thrust < auto_stage.available_thrust:
        vessel.control.activate_next_stage()
        logger.info("Staging")
        auto_stage.available_thrust = vessel.available_thrust - 10


def countdown(seconds):
    """Display a simple countdown to launch

    Parameters:
    `seconds`: the number of ticks in the countdown
    """
    for t in range(seconds, 0, -1):
        logger.info(f"{t}...")
        time.sleep(1)
    logger.info("Launch!")


def do_prelaunch():
    """Pre-launch setup"""
    vessel.control.sas = True
    vessel.control.sas_mode = SASMode.stability_assist
    vessel.control.rcs = False
    vessel.control.throttle = 1.0


def do_launch(seconds, heading=90):
    """Launch the rocket after `seconds` countdown, pointing in `heading` direction

    Parameters:
    `seconds`: Time until launch in seconds
    `heading`: The direction to take off in
    """

    # Countdown...
    countdown(5)

    # Activate the first stage
    vessel.auto_pilot.engage()
    vessel.auto_pilot.target_pitch_and_heading(90, heading)
    # Stage until the rocket gets some thrust
    while vessel.thrust < 10:
        vessel.control.activate_next_stage()
        time.sleep(1)


def do_ascent(target_altitude=100000, target_heading=90):
    """Perform an ascent, pitching over as the eocket climbs.

    Parameters:
    `target_altitude`: the target altituide to reach
    `target_heading`: the heading to use when ascending
    """
    logger.info(f"Ascending to apoapsis {target_altitude} on heading {target_heading}")

    # Set the initial parameters, heading, pitch and roll
    vessel.auto_pilot.target_heading = target_heading
    vessel.auto_pilot.target_pitch = 90
    if FEATURES["PERFORM_ASCENT_ROLL"]:
        vessel.auto_pilot.target_roll = 0
    else:
        vessel.auto_pilot.target_roll = 90

    with conn.stream(vessel.orbit, "apoapsis_altitude") as apoapsis, conn.stream(
        vessel.flight(), "surface_altitude"
    ) as altitude, conn.stream(vessel.orbit, "time_to_apoapsis") as time_to_apoapsis:

        while apoapsis() < target_altitude * 0.95:

            # Get rid of any used stages
            auto_stage()

            # Adjust the pitch
            vessel.auto_pilot.target_pitch = max(
                88.963 - 1.03287 * altitude() ** 0.409511, 0
            )

            if not FEATURES["FULL_BURN"]:
                # Apply a simple change to thrust based on time to apoapsis
                if time_to_apoapsis() < 60:
                    vessel.control.throttle = 1.0
                if time_to_apoapsis() > 65 and time_to_apoapsis() < 120:
                    vessel.control.throttle = 0.5
                if time_to_apoapsis() > 125:
                    vessel.control.throttle = 0.25

        vessel.control.throttle = 0.1
        while apoapsis() < target_altitude:
            pass

    logger.info("Target apoapsis reached")
    vessel.control.throttle = 0.0


def do_coast(altitude):
    """Coast until out of the atmosphere, keeping apoapsis at `altitude`

    Parameters:
    `altitude`: the target altitude to hold
    """
    logger.info(f"Coasting out of atmosphere to apoapsis of {altitude}")

    vessel.auto_pilot.target_pitch = 0

    coast_pid = pid.PID(P=0.3, I=0, D=0)
    coast_pid.setpoint(altitude)

    # Until we exit the planets atmosphere
    atmosphere_depth = vessel.orbit.body.atmosphere_depth
    with conn.stream(vessel.flight(), "mean_altitude") as altitude:
        with conn.stream(vessel.orbit, "apoapsis_altitude") as apoapsis:
            while altitude() < atmosphere_depth:
                vessel.control.throttle = limit(
                    vessel.control.throttle + coast_pid.update(apoapsis())
                )
                auto_stage
                time.sleep(0.1)


def set_sas_mode(new_mode):
    """Set an SAS mode, if all goes well returns True, if any exception is thrown then return False"""
    try:
        vessel.control.sas = True
        time.sleep(0.1)
        vessel.control.sas_mode = new_mode
        return True
    except:
        return False


def convert_time(seconds):
    """Convert a number of seconds to days:hours:minutes:seconds string"""
    m, s = divmod(seconds, 60)
    h, m = divmod(m, 60)
    d, h = divmod(h, 6)
    return f"{int(d)}:{int(h)}:{int(m)}:{s:f}"


def show_orbit():
    """Show current orbit parameters"""
    orbit = vessel.orbit
    s = ""
    s += f"\nOrbiting {orbit.body.name}"
    s += f"\nApoapsis = {orbit.apoapsis_altitude}"
    s += f"\nPeriapsis = {orbit.periapsis_altitude}"
    s += f"\nEccentricty = {orbit.eccentricity}"
    s += f"\nInclination = {math.degrees(orbit.inclination)}"
    s += f"\nPeriod = {convert_time(orbit.period)}"

    logger.info(s)


def hill_climb(data, score_function):
    """Modify the list of data and use the score_function to improve it

    Parameters:
    `data`: a list of data points that can be manipulated
    `score_function`: a function that can take the data list and return a score, lower score is better
    """
    current_score = score_function(data)
    current_best = data.copy()
    for step in [100, 10, 1, 0.1]:
        improved = True
        while improved:
            # Create a list of candidate data lists with each elements adjusted by +-step
            candidates = []
            for c in range(0, len(data)):
                inc = current_best.copy()
                dec = current_best.copy()
                inc[c] += step
                dec[c] -= step
                candidates += [inc, dec]
            # Score each candidate
            improved = False
            for c in candidates:
                score = score_function(c)
                if score < current_score:
                    current_score = score
                    current_best = c
                    improved = True
    return current_best


def score_eccenticity(data):
    """Score function for the hill climb to circularise an orbit

    Only need to tweak the prograde as the time will always be at apoapsis and
    to circularise only requires a prograde burn
    Parameters:
    `data`: list with single item - prograde deltaV burn
    """
    node = vessel.control.add_node(
        ksc.ut() + vessel.orbit.time_to_apoapsis, prograde=data[0]
    )
    score = node.orbit.eccentricity
    node.remove()
    return score


def calculate_burn_time(node):
    # Calculate burn time (using rocket equation)
    f = vessel.available_thrust
    isp = vessel.specific_impulse * G0
    m0 = vessel.mass
    m1 = m0 / math.exp(node.delta_v / isp)
    flow_rate = f / isp
    return (m0 - m1) / flow_rate


###################################################
#
# Go to space
#
###################################################

show_features()
do_prelaunch()
do_launch(5, TARGET_HEADING)
do_ascent(TARGET_ALTITUDE, TARGET_HEADING)
do_coast(TARGET_ALTITUDE)

delta_v = hill_climb([0], score_eccenticity)
# Create the circularisation node at apoapsis
node = vessel.control.add_node(
    ksc.ut() + vessel.orbit.time_to_apoapsis, prograde=delta_v
)
burn_time = calculate_burn_time(node)

# Orientate ship
logger.info("Orientating ship for circularization burn")
if FEATURES["USE_SAS"]:
    vessel.auto_pilot.disengage()
    if set_sas_mode(SASMode.maneuver):
        logger.info("Using SAS")
        pointing_at_node = False
        n = Vector(0, 1, 0)
        with conn.stream(vessel.flight(node.reference_frame), "direction") as facing:
            while not pointing_at_node:
                d = Vector(facing())
                if d.angle(n) < 2:
                    pointing_at_node = True
                time.sleep(0.1)
        logger.info("Returning to Auto Pilot")
    else:
        logger.info("Using auto pilot")

vessel.auto_pilot.sas = False
vessel.auto_pilot.reference_frame = node.reference_frame
vessel.auto_pilot.target_direction = (0, 1, 0)
vessel.auto_pilot.engage()
vessel.auto_pilot.wait()

# Wait until burn
logger.info("Waiting until circularization burn")
burn_ut = ksc.ut() + vessel.orbit.time_to_apoapsis - (burn_time / 2.0)
lead_time = 5
ksc.warp_to(burn_ut - lead_time)

# Execute burn
with conn.stream(vessel.orbit, "time_to_apoapsis") as time_to_apoapsis:
    while time_to_apoapsis() - (burn_time / 2.0) > 0:
        pass
logger.info("Executing burn")

with conn.stream(node, "remaining_delta_v") as remaining_delta_v:
    dv = remaining_delta_v()  # prograde only
    last_dv = dv
    while dv > 0.01:
        engine_dv = vessel.available_thrust / vessel.mass
        vessel.control.throttle = max(min((dv / engine_dv), 1), 0.01)
        last_dv = dv
        dv = remaining_delta_v()
        if dv > last_dv + 0.01:
            break

vessel.control.throttle = 0.0
node.remove()

logger.info("Launch complete")

time.sleep(10)
show_orbit()

logger.critical("PROGRAM ENDED")
