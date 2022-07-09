"""Launch a rocket"""

import time

from krpcutils import KrpcUtilities
from circularise import Circularise
from features import Features
from logger import Logger
from connection import Connection

# Some constants
DEFAULT_ALTITUDE = 100_000
DEFAULT_INCLINATION = 0.0

###################################################
#
# Go to space
#
###################################################
def launch_rocket(features, logger, utils, circularise):
    """Launch the rocket"""

    features.show_features()
    utils.wait_for_LAN(features.get("TARGET_LAN"))
    utils.do_prelaunch()
    utils.do_launch()
    if features.exists("TEST_ALTITUDE") and features.exists("TEST_SPEED"):
        utils.do_test(features.get("TEST_ALTITUDE"), features.get("TEST_SPEED"))
    else:
        utils.do_ascent(
            features.get("TARGET_ALTITUDE", DEFAULT_ALTITUDE),
            features.get("TARGET_INCLINATION", DEFAULT_INCLINATION),
            perform_roll=features.get("PERFORM_ASCENT_ROLL", True),
            full_burn=features.get("FULL_BURN", False),
        )
        utils.do_coast(features.get("TARGET_ALTITUDE", DEFAULT_ALTITUDE))
        utils.jettison_fairings()
        time.sleep(5)
        utils.extend_solar_panels()
        utils.extend_antennas()
        circularise.do_circularisation(features.get("USE_SAS"))

    logger.info("Launch complete")

    logger.info("PROGRAM ENDED")


if __name__ == "__main__":
    # Configurable Features
    feature_set = Features()

    # Launch parameters
    # feature_set.set("TARGET_ALTITUDE", 100_000)
    # feature_set.set("TARGET_INCLINATION", 116.6)
    # feature_set.set("TARGET_LAN", 154.6)

    # Test parameters
    # feature_set.set("TEST_ALTITUDE", 41_000)
    # feature_set.set("TEST_SPEED", 480)

    # Configure objects

    my_logger = Logger().get()
    my_connection = Connection(logger=my_logger, name="Launch into orbit")
    my_utils = KrpcUtilities(logger=my_logger, connection=my_connection)
    my_circularise = Circularise(
        logger=my_logger, connection=my_connection, utils=my_utils
    )

    launch_rocket(
        features=feature_set,
        logger=my_logger,
        utils=my_utils,
        circularise=my_circularise,
    )
