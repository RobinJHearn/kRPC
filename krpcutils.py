#
# Class of useful utility functions
#
import logging
import math
import time

import krpc

from decorators import singleton
from vector import Vector

logger = logging.getLogger(__name__)

# Universal Constants
G0 = 9.80665


@singleton
class kRPC_Utilities(object):
    def __init__(self, connection):
        self.conn = connection
        self.ksc = self.conn.space_center  # pylint: disable=no-member
        self.vessel = self.ksc.active_vessel
        self.SASMode = self.ksc.SASMode

    def auto_stage(self):
        """Stage engines when thrust drops"""
        if not hasattr(self, "available_thrust"):
            self.available_thrust = self.vessel.available_thrust - 10
        if self.vessel.available_thrust < self.available_thrust:
            while True:
                self.vessel.control.activate_next_stage()
                logger.info("Staging")
                if self.vessel.available_thrust > 0:
                    break
            self.available_thrust = self.vessel.available_thrust - 10

    def wait_until_time(self, ut_time, use_warp=True):
        """Wait until it is specified absolute time

        Parameters:
        `ut_time`: Absolute universal time in seconds to wait until
        """
        logger.info("Waiting until time")
        if use_warp:
            lead_time = 5
            self.ksc.warp_to(ut_time - lead_time)
        # Wait for the last few seconds
        while ut_time - self.ksc.ut > 0:
            pass

    def set_sas_mode(self, new_mode):
        """Set an SAS mode, if all goes well returns True, if any exception is thrown then return False"""
        try:
            self.vessel.control.sas = True
            time.sleep(0.1)
            self.vessel.control.sas_mode = new_mode
            return True
        except:
            return False

    def calculate_burn_time(self, node):
        # Calculate burn time (using rocket equation)
        f = self.vessel.available_thrust
        isp = self.vessel.specific_impulse * G0
        m0 = self.vessel.mass
        m1 = m0 / math.exp(node.delta_v / isp)
        flow_rate = f / isp
        return (m0 - m1) / flow_rate

    def align_with_node(self, node, use_sas):
        """Align the ship with the burn vector of the node

        Parameters:
        `node`: The node to align with
        """
        logger.info("Orientating ship for burn")
        if use_sas:
            self.vessel.auto_pilot.disengage()
            if self.set_sas_mode(self.SASMode.maneuver):
                logger.info("Using SAS")
                pointing_at_node = False
                n = Vector(0, 1, 0)
                with self.conn.stream(
                    getattr, self.vessel.flight(node.reference_frame), "direction"
                ) as facing:
                    while not pointing_at_node:
                        d = Vector(facing())
                        if d.angle(n) < 2:
                            pointing_at_node = True
                        time.sleep(0.1)
                logger.info("Returning to Auto Pilot")
            else:
                logger.info("Using auto pilot")

        self.vessel.auto_pilot.sas = False
        self.vessel.auto_pilot.reference_frame = node.reference_frame
        self.vessel.auto_pilot.target_direction = (0, 1, 0)
        self.vessel.auto_pilot.engage()
        self.vessel.auto_pilot.wait()

    def execute_burn(self, node):
        """Execute the burn defined by the node, assume the craft is already aligned for the burn

        Parameters:
        `node`: Node defining the deltaV needed for the burn
        """
        logger.info("Executing burn")

        with self.conn.stream(getattr, node, "remaining_delta_v") as remaining_delta_v:
            dv = remaining_delta_v()
            last_dv = dv
            while dv > 0.01:
                self.auto_stage()
                engine_dv = self.vessel.available_thrust / self.vessel.mass
                if engine_dv > 0:
                    self.vessel.control.throttle = max(min((dv / engine_dv), 1), 0.01)
                last_dv = dv
                dv = remaining_delta_v()
                if dv > last_dv + 0.01:
                    break
        self.vessel.control.throttle = 0.0

    def execute_next_node(self, use_sas):
        """Execute the next node and then remove it from the plan"""
        if len(self.vessel.control.nodes) > 0:
            node = self.vessel.control.nodes[0]

            burn_time = self.calculate_burn_time(node)
            self.align_with_node(node, use_sas)
            self.wait_until_time(node.ut - (burn_time / 2))
            self.execute_burn(node)

            node.remove()
        else:
            logger.error("execute_next_node: No Node to execute")

    def add_node(self, data):
        """Add a node and return it based on the data list

        Parameters:
        `data`: List of node values = time, prograde, radial and normal burns
        As a minimum time must be present
        """
        prograde = data[1] if len(data) > 1 else 0
        radial = data[2] if len(data) > 2 else 0
        normal = data[3] if len(data) > 3 else 0

        node = self.vessel.control.add_node(
            ut=data[0], prograde=prograde, radial=radial, normal=normal
        )
        return node
