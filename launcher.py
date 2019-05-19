import logging
import math
import time

import krpc

import krpclogstream
import krpcutils
import pid
from decorators import log_as, log_debug
from hillclimb import hill_climb
from vector import Vector

# Configurable Features
FEATURES = {
    "FULL_BURN": False,  # Don't reduce throttle on ascent burn
    "USE_SAS": True,  # Use SAS to point for maneuver
    "PERFORM_ASCENT_ROLL": False,  # Roll rocket on launch ala NASA
    "SHOW_LOG_IN_KSP": False,  # Show log in a window in KSP
}

# Launch parameters
TARGET_ALTITUDE = 100_000
TARGET_HEADING = 90

# Configure connection to KSP and some short cuts
conn = krpc.connect(name="Launch into orbit")
ksc = conn.space_center  # pylint: disable=no-member
vessel = ksc.active_vessel
SASMode = ksc.SASMode

# Create the utility package
utils = krpcutils.kRPC_Utilities(conn)

# Configure logging
logging.basicConfig(format="%(asctime)-15s %(levelname)s %(message)s")
logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)
if FEATURES["SHOW_LOG_IN_KSP"]:
    ksp_stream = krpclogstream.KrpcLogStream(conn)
    handler = logging.StreamHandler(ksp_stream)
    logger.addHandler(handler)


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


def do_launch():
    """Launch the rocket pointing straight up"""

    # Activate the first stage
    vessel.auto_pilot.engage()
    vessel.auto_pilot.target_pitch_and_heading(90, 90)
    # Stage until the rocket gets some thrust
    while vessel.thrust < 10:
        vessel.control.activate_next_stage()
        time.sleep(1)


def do_ascent(target_altitude=100_000, target_heading=90):
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

    with conn.stream(
        getattr, vessel.orbit, "apoapsis_altitude"
    ) as apoapsis, conn.stream(
        getattr, vessel.flight(), "surface_altitude"
    ) as altitude, conn.stream(
        getattr, vessel.orbit, "time_to_apoapsis"
    ) as time_to_apoapsis:

        while apoapsis() < target_altitude * 0.95:

            # Get rid of any used stages
            utils.auto_stage()

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
    with conn.stream(
        getattr, vessel.flight(), "mean_altitude"
    ) as altitude, conn.stream(getattr, vessel.orbit, "apoapsis_altitude") as apoapsis:
        while altitude() < atmosphere_depth:
            vessel.control.throttle = limit(
                vessel.control.throttle + coast_pid.update(apoapsis())
            )
            utils.auto_stage()
            time.sleep(0.1)


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


def score_eccenticity(data):
    """Score function for the hill climb to circularise an orbit

    Only need to tweak the prograde as the time will always be at apoapsis and
    to circularise only requires a prograde burn
    Parameters:
    `data`: list with single item - prograde deltaV burn
    """
    node = utils.add_node([ksc.ut + vessel.orbit.time_to_apoapsis, data[0]])
    score = node.orbit.eccentricity
    node.remove()
    return score


def do_circularisation():
    """Work out and execute the circularization burn

    Only need to adjust the prograde burn as it will always be at the apoapsis
    """
    logger.info("Circularising orbit")

    delta_v = hill_climb([0], score_eccenticity)[0]
    utils.add_node([ksc.ut + vessel.orbit.time_to_apoapsis, delta_v])
    utils.execute_next_node(FEATURES["USE_SAS"])


def score_inclination(target_inclination):
    """Wrapper to set a target inclination value"""

    def score_function(data):
        """Scoring function for inclination changes

        Parameters:
        `data`: list of four items representing time, prograde, radial and normal burns
        """
        node = utils.add_node([data[0], 0, 0, data[1]])
        inclination = math.degrees(node.orbit.inclination)
        logger.debug(f"target inc: {target_inclination} node.inc: {inclination}")
        score = abs(inclination - target_inclination)
        node.remove()
        return score

    return score_function


def inclination_change(inclination):
    """Change inclination of orbit

    Inclination change only requires a normal burn (preferrably at an
    ascending or descending node)
    """
    logger.info(f"Changing orbit inclination to {inclination} degrees")

    score_function = score_inclination(inclination)
    data = hill_climb([ksc.ut + 60, 0], score_function)
    utils.add_node([data[0], 0, 0, data[1]])


#    utils.execute_next_node(FEATURES["USE_SAS"])


###################################################
#
# Go to space
#
###################################################

# show_features()
# do_prelaunch()
# countdown(5)
# do_launch()
# do_ascent(TARGET_ALTITUDE, TARGET_HEADING)
# do_coast(TARGET_ALTITUDE)
# do_circularisation()

# logger.info("Launch complete")

# # Wait for the orbit to settle
# time.sleep(2)
# show_orbit()

while True:
    # Ascending node is when mean anomaly = PI/2 (90 degrees)
    # Descending node is when mean anomaly = 3PI/2 (270 degrees)
    # Mean anomaly =0 at periapsis, PI at apoapsis
    M1 = (math.pi / 2) - vessel.orbit.eccentricity
    M2 = (3 * math.pi / 2) - vessel.orbit.eccentricity
    theta = math.degrees(vessel.orbit.true_anomaly)
    if theta > 270 or theta < 90:
        Mx = M1
        node = "M1"
    else:
        Mx = M2
        node = "M2"
    P = vessel.orbit.period
    n = 2 * math.pi / P
    M = vessel.orbit.mean_anomaly
    t1 = (M1 - M) / n
    t2 = (M2 - M) / n
    print(
        f"Time to next node M1 = {convert_time(t1)} seconds, M2 = {convert_time(t2)} M = {math.degrees(M)}"
    )
    time.sleep(1)

inclination_change(15)

logger.info("PROGRAM ENDED")
