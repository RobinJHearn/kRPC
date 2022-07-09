"""Class to circulise an orbit"""

from hillclimb import hill_climb


class Circularise(object):
    """Circularise an orbit"""

    def __init__(self, connection, utils, logger):
        self.conn = connection
        self.utils = utils
        self.logger = logger

    def score_eccenticity(self, data):
        """Score function for the hill climb to circularise an orbit

        Only need to tweak the prograde as the time will always be at apoapsis and
        to circularise only requires a prograde burn
        Parameters:
        `data`: list with single item - prograde deltaV burn
        """
        vessel = self.conn.vessel()
        if vessel:
            node = self.utils.add_node(
                [
                    self.conn.space_center().ut + vessel.orbit.time_to_apoapsis,
                    data[0],
                ]
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
        self.logger.info("Circularising orbit")

        if self.conn.vessel():
            delta_v = hill_climb([0], self.score_eccenticity)[0]
            self.utils.add_node(
                [
                    self.conn.space_center().ut + self.vessel.orbit.time_to_apoapsis,
                    delta_v,
                ]
            )
            self.utils.execute_next_node(use_sas)
        else:
            self.logger.info("No vessel to circularise")
