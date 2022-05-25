"""Launch a rocket"""
import logging
import math
import time

import krpc

import krpclogstream
import krpcutils
import circularise
import pid
from decorators import log_debug
from hillclimb import hill_climb

# Configurable Features
FEATURES = {
    "FULL_BURN": False,  # Don't reduce throttle on ascent burn
    "USE_SAS": True,  # Use SAS to point for maneuver
    "PERFORM_ASCENT_ROLL": True,  # Roll rocket on launch ala NASA
    "SHOW_LOG_IN_KSP": False,  # Show log in a window in KSP
    "TEST_AT": False,  # Implement a straight up test of a component at alt/speed
    "AT_LAN": False,  # Launch at the specified LAN
}

# Launch parameters
TARGET_ALTITUDE = 80_000
TARGET_INCLINATION = 0
TARGET_LAN = 56.7

TEST_ALTITUDE = 41_000
TEST_SPEED = 480

# Configure connection to KSP and some short cuts
conn = krpc.connect(name="Launch into orbit")
ksc = conn.space_center  # pylint: disable=no-member
vessel = ksc.active_vessel
SASMode = ksc.SASMode

# Configure logging
logging.basicConfig(format="%(asctime)-15s %(levelname)s %(message)s")
logger = logging.getLogger("KSP")
logger.setLevel(logging.INFO)
if FEATURES["SHOW_LOG_IN_KSP"]:
    ksp_stream = krpclogstream.KrpcLogStream(conn)
    handler = logging.StreamHandler(ksp_stream)
    logger.addHandler(handler)

# Create the utility package
utils = krpcutils.KrpcUtilities(conn)
circularise = circularise.Circularise(conn)


def show_features():
    """Log out the configurable features and their state"""
    for key, value in FEATURES.items():
        logger.info("%s -> %s", key, value)


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


def do_ascent(target_altitude=100_000, target_inclination=0):
    """Perform an ascent, pitching over as the rocket climbs.

    Parameters:
    `target_altitude`: the target altituide to reach
    `target_heading`: the heading to use when ascending
    """
    logger.info(
        "Ascending to apoapsis %s} on heading %s", target_altitude, target_inclination
    )

    # Set the initial parameters, heading, pitch and roll
    heading = utils.launch_heading(target_inclination, target_altitude)
    logger.info("Target heading %s", heading)
    vessel.auto_pilot.target_heading = heading
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

            logger.debug("Current Heading %s", vessel.auto_pilot.target_heading)

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
    logger.info("Coasting out of atmosphere to apoapsis of %s", altitude)

    vessel.auto_pilot.target_pitch = 0

    coast_pid = pid.PID(P=0.3, I=0, D=0)
    coast_pid.setpoint(altitude)

    # Until we exit the planets atmosphere
    atmosphere_depth = vessel.orbit.body.atmosphere_depth
    with conn.stream(
        getattr, vessel.flight(), "mean_altitude"
    ) as altitude, conn.stream(getattr, vessel.orbit, "apoapsis_altitude") as apoapsis:
        while altitude() < atmosphere_depth:
            vessel.control.throttle = utils.limit(
                vessel.control.throttle + coast_pid.update(apoapsis())
            )
            utils.auto_stage()
            time.sleep(0.1)


def score_inclination(target_inclination, target_time):
    """Wrapper to set a target inclination value"""

    @log_debug(logger)
    def score_function(data):
        """Scoring function for inclination changes

        Parameters:
        `data`: list of three items representing prograde, radial and normal burns
        """
        node = utils.add_node([target_time, 0, 0, data[0]])
        inclination = math.degrees(node.orbit.inclination)
        logger.debug("target inc: %s node.inc: %s", target_inclination, inclination)
        score = abs(target_inclination - inclination)
        node.remove()
        return score

    return score_function


def inclination_change(inclination):
    """Change inclination of orbit

    Inclination change only requires a normal burn (preferrably at an
    ascending or descending node)
    """
    logger.info("Changing orbit inclination to %s degrees", inclination)

    (an_time, dn_time) = utils.time_ascending_descending_nodes(delta_time=False)
    # Which node do we want
    if inclination > math.degrees(vessel.orbit.inclination):
        # Going down so descending node
        node_time = dn_time
    else:
        node_time = an_time

    score_function = score_inclination(inclination, node_time)
    data = hill_climb([0], score_function)[0]
    utils.add_node([node_time, 0, 0, data])


def do_test(test_altitude, test_speed):
    """Test a component at an altitude/speed"""

    logger.info("Target Alt: %s, Target Speed: %s", test_altitude, test_speed)
    srf_frame = vessel.orbit.body.reference_frame

    with conn.stream(
        getattr, vessel.flight(), "mean_altitude"
    ) as altitude, conn.stream(getattr, vessel.flight(srf_frame), "speed") as speed:
        while altitude() < test_altitude:
            logger.info("Altitude: %s, Speed: %s", altitude(), speed())
            if speed() > 1.1 * test_speed:
                vessel.control.throttle = 0.0
            if speed() < 0.9 * test_speed:
                vessel.control.throttle = 1.0
            time.sleep(1.0)

        # Run the test
        for part in vessel.parts.all:
            for module in part.modules:
                if module.has_event("Run Test"):
                    module.trigger_event("Run Test")


###################################################
#
# Go to space
#
###################################################

show_features()
do_prelaunch()
utils.countdown(5)
do_launch()
if FEATURES["TEST_AT"]:
    do_test(TEST_ALTITUDE, TEST_SPEED)
else:
    do_ascent(TARGET_ALTITUDE, TARGET_INCLINATION)
    do_coast(TARGET_ALTITUDE)
    utils.jettison_fairings()
    time.sleep(5)
    utils.extend_solar_panels()
    utils.extend_antennas()
    circularise.do_circularisation(FEATURES["USE_SAS"])

logger.info("Launch complete")

logger.info("PROGRAM ENDED")
