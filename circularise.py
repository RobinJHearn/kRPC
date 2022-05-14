"""Class to circulise an orbit"""

import logging
from hillclimb import hill_climb
import krpcutils


from decorators import singleton

logger = logging.getLogger(__name__)


@singleton
class Circularise(object):
    """Circularise an orbit"""

    def __init__(self, connection):
        self.conn = connection
        self.ksc = self.conn.space_center  # pylint: disable=no-member
        self.vessel = self.ksc.active_vessel
        self.utils = krpcutils.KrpcUtilities(self.conn)

    def score_eccenticity(self, data):
        """Score function for the hill climb to circularise an orbit

        Only need to tweak the prograde as the time will always be at apoapsis and
        to circularise only requires a prograde burn
        Parameters:
        `data`: list with single item - prograde deltaV burn
        """
        node = self.utils.add_node(
            [self.ksc.ut + self.vessel.orbit.time_to_apoapsis, data[0]]
        )
        score = node.orbit.eccentricity
        node.remove()
        return score

    def do_circularisation(self, use_sas):
        """Work out and execute the circularization burn

        Only need to adjust the prograde burn as it will always be at the apoapsis
        Parameters:
        `use_sas`: Boolean to indicate if SAS should be used to align to node
        """
        logger.info("Circularising orbit")

        delta_v = hill_climb([0], self.score_eccenticity)[0]
        self.utils.add_node([self.ksc.ut + self.vessel.orbit.time_to_apoapsis, delta_v])
        self.utils.execute_next_node(use_sas)
