""" Common logger """

import logging


class Logger:
    """A common logger that can be used by other classes"""

    def __init__(self):
        # Configure logging
        logging.basicConfig(format="%(asctime)-15s %(levelname)s %(message)s")
        self.logger = logging.getLogger("KSP")
        self.logger.setLevel(logging.INFO)

    def get(self):
        """Return the logger object"""
        return self.logger
