"""
Class holding the features of a launch
"""

import logging


class Features:
    """Holds a set of features"""

    def __init__(self):
        """Default features"""
        self.logger = logging.getLogger("KSP")
        self.features = {}
        # self.set("ALTITUDE", 100_000)  # Default altitude to launch to
        # self.set("FULL_BURN", False)  # Reduce burn during launch
        # self.set("USE_SAS", True)  # Use SAS to point for maneuver
        # self.set("PERFORM_ASCENT_ROLL", True)  # Roll rocket on launch ala NASA
        # self.set("SHOW_LOG_IN_KSP", False)  # Show log in a window in KSP
        # self.set(
        #     "TEST_AT", False
        # )  # Implement a straight up test of a component at alt/speed
        # self.set("AT_LAN", False)  # Launch at the specified LAN

    def set(self, feature, value):
        """Set a feature"""
        self.features[feature] = value

    def get(self, feature, default = None):
        """Get a feature or return default if it doesn't exist"""
        return self.features.get(feature, default)

    def exists(self, feature):
        """Test if a feature has been set"""
        return feature in self.features

    def show_features(self):
        """Log out the configurable features and their state"""
        for key, value in self.features.items():
            self.logger.info("%s -> %s", key, value)
