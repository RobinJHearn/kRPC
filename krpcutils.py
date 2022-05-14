"""
Class of useful utility functions
"""
import logging
import math
import time

import krpc

from decorators import singleton
from vector import Vector

logger = logging.getLogger(__name__)

# Universal Constants
G0 = 9.80665
HOURS_PER_DAY = 6
SECONDS_PER_DEGREE = 60


@singleton
class KrpcUtilities(object):
    """Some useful utility functions"""

    def __init__(self, connection):
        self.conn = connection
        self.ksc = self.conn.space_center  # pylint: disable=no-member
        self.vessel = self.ksc.active_vessel
        self.sas_mode = self.ksc.SASMode
        self.available_thrust = 0

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

    def jettison_fairings(self):
        """Jettison all fairings"""
        for fairing in self.vessel.parts.fairings:
            fairing.jettison()

    def extend_solar_panels(self):
        """Extend all solar panels"""
        for solar_panel in self.vessel.parts.solar_panels:
            if solar_panel.deployable:
                solar_panel.deployed = True

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
        """Set an SAS mode, if all goes well returns True,
        if any exception is thrown then return False"""
        try:
            self.vessel.control.sas = True
            time.sleep(0.1)
            self.vessel.control.sas_mode = new_mode
            return True
        except:  # pylint: disable=bare-except
            return False

    def calculate_burn_time(self, node):
        """Calculate burn time for a node"""
        # Calculate burn time (using rocket equation)
        force = self.vessel.available_thrust
        isp = self.vessel.specific_impulse * G0
        m0_ = self.vessel.mass
        m1_ = m0_ / math.exp(node.delta_v / isp)
        flow_rate = force / isp
        return (m0_ - m1_) / flow_rate

    def align_with_node(self, node, use_sas):
        """Align the ship with the burn vector of the node

        Parameters:
        `node`: The node to align with
        """
        logger.info("Orientating ship for burn")
        if use_sas:
            self.vessel.auto_pilot.disengage()
            if self.set_sas_mode(self.sas_mode.maneuver):
                logger.info("Using SAS")
                pointing_at_node = False
                normal = Vector(0, 1, 0)
                with self.conn.stream(
                    getattr, self.vessel.flight(node.reference_frame), "direction"
                ) as facing:
                    while not pointing_at_node:
                        direction = Vector(facing())
                        if direction.angle(normal) < 2:
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
            dv_ = remaining_delta_v()
            last_dv = dv_
            while dv_ > 0.01:
                self.auto_stage()
                engine_dv = self.vessel.available_thrust / self.vessel.mass
                if engine_dv > 0:
                    self.vessel.control.throttle = max(min((dv_ / engine_dv), 1), 0.01)
                last_dv = dv_
                dv_ = remaining_delta_v()
                if dv_ > last_dv + 0.01:
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

    def time_ascending_descending_nodes(self, delta_time=False):
        """Calculate time to/of ascending and descending nodes

        Parameters:
        `delta_time`: Boolean if True a time until is returned,
                      if False the an absolute time is returned
        Returns:
        Tuple containing (Time to Ascending Node, Time To Descending Node)
        """
        # Argument of periapsis is angle from ascending node to periapsis
        ap_ = self.vessel.orbit.argument_of_periapsis
        tau = math.pi * 2
        period = self.vessel.orbit.period

        # Convert to angle FROM periapsis to AN
        peri_to_an = tau - ap_
        # DN is opposite AN
        peri_to_dn = (
            peri_to_an - math.pi if peri_to_an >= math.pi else peri_to_an + math.pi
        )

        # Calculate delta time so we can mod by the period to get the shortest time from now
        time_now = self.ksc.ut
        time_an = (self.vessel.orbit.ut_at_true_anomaly(peri_to_an) - time_now) % period
        time_dn = (self.vessel.orbit.ut_at_true_anomaly(peri_to_dn) - time_now) % period

        if not delta_time:
            time_an = time_an + time_now
            time_dn = time_dn + time_now

        return (time_an, time_dn)

    def limit_absolute_angle(self, angle):
        """Limit an angle to absolue value 0..360

        Parameters:
        `angle`: Angle in degrees to limt
        """
        while angle < 0:
            angle += 360
        while angle >= 360:
            angle -= 360
        return angle

    def limit(self, value, min_value=0, max_value=1):
        """Limit a `value` to between the `minValue` and `maxValue` inclusive

        Parameters:
        `value`: value to limit
        `min_value`: minimum value allowed
        `max_value: maximum value allowed
        """
        return min(max(value, min_value), max_value)

    #
    #  function calculateInclination {
    #    parameter inclination, orbitAltitude.
    #
    #    set inclination to mod(inclination + 360, 360).               // Convert to 0..360
    #    local cosAngle is cos(inclination)/cos(ship:latitude).
    #    if abs(cosAngle) > 1 if inclination < 90 or inclination > 270 return 90. else return 270.
    #    local launchAngle is arcsin(cosAngle).
    #    if inclination > 180 set launchAngle to 180 - launchAngle.

    #    local binertial is mod(launchAngle + 360, 360).                           // 0..360
    #    local binertial is headingForInclination(inclination).
    #    local vorbit is sqrt(ship:body:mu/(ship:body:radius + orbitAltitude)).
    #    local veqrot is (2 * constant:PI * ship:body:radius) / ship:body:rotationperiod.
    #    local vxrot is vorbit * sin(binertial) - veqrot * cos(ship:latitude).
    #    local vyrot is vorbit * cos(binertial).
    #    local brot is arctan(vxrot/vyrot).
    #    if inclination >= 180 set brot to 180 + brot.
    #    return brot.
    #  }

    def launch_heading(self, inclination, orbit_altitude):
        """Calculate the launch heading to achive a desired orbital inclination

        Parameters:
        `inclination`: Desired inclination in degrees
        `orbitAltitude`: Height in metres above the surface of the desired orbit
        """

        logger.debug(
            "Requested Inclination %s, altitude %s", inclination, orbit_altitude
        )
        # Make an absolute inclination (0 = East, 90 = North)
        abs_inclination = self.limit_absolute_angle(inclination)
        # Account for launch site latitude, the inclination must be less than the launch
        # site latitude
        site_latitude = self.vessel.flight().latitude
        # Convert so 0 is east
        cos_inclination = math.cos(math.radians(abs_inclination))
        cos_launch_site = math.cos(math.radians(site_latitude))
        if math.fabs(abs_inclination) >= math.fabs(site_latitude):
            cos_angle = self.limit(cos_inclination / cos_launch_site, -1, 1)
            launch_angle = math.degrees(math.asin(cos_angle))
            if abs_inclination >= 180:
                launch_angle = 180 - launch_angle
        else:
            if cos_inclination >= 0:
                launch_angle = 90
            else:
                launch_angle = 270

        logger.debug("Launch Angle %s", launch_angle)

        mu_ = self.ksc.g * self.vessel.orbit.body.mass
        body_radius = self.vessel.orbit.body.equatorial_radius
        orbit_from_centre = body_radius + orbit_altitude
        v_orbit = math.sqrt(mu_ / orbit_from_centre)
        v_equator = (
            2 * math.pi * body_radius
        ) / self.vessel.orbit.body.rotational_period
        v_x = (v_orbit * math.sin(math.radians(launch_angle))) - (
            v_equator * cos_launch_site
        )
        v_y = v_orbit * math.cos(math.radians(launch_angle))
        b_rot = math.degrees(math.atan(v_x / v_y))
        if abs_inclination >= 180:
            b_rot = 180 + b_rot
        logger.debug("Launch Heading %s", b_rot)

        #    local vorbit is sqrt(ship:body:mu/(ship:body:radius + orbitAltitude)).
        #    local veqrot is (2 * constant:PI * ship:body:radius) / ship:body:rotationperiod.
        #    local vxrot is vorbit * sin(binertial) - veqrot * cos(ship:latitude).
        #    local vyrot is vorbit * cos(binertial).
        #    local brot is arctan(vxrot/vyrot).
        #    if inclination >= 180 set brot to 180 + brot.
        #    return brot.

        return b_rot

    def countdown(self, seconds):
        """Display a simple countdown to launch

        Parameters:
        `seconds`: the number of ticks in the countdown
        """
        for tick in range(seconds, 0, -1):
            logger.info("%s...", tick)
            tick.sleep(1)
        logger.info("Launch!")

    def convert_time(self, seconds):
        """Convert a number of seconds to days:hours:minutes:seconds string"""
        mins, secs = divmod(seconds, 60)
        hours, mins = divmod(mins, 60)
        days, hours = divmod(hours, 6)
        return f"{int(days)}:{int(hours):02}:{int(mins):02}:{secs:02.2f}"

    def show_orbit(self):
        """Show current orbit parameters"""
        orbit = self.vessel.orbit
        string = ""
        string += f"\nOrbiting {orbit.body.name}"
        string += f"\nApoapsis = {orbit.apoapsis_altitude}"
        string += f"\nPeriapsis = {orbit.periapsis_altitude}"
        string += f"\nEccentricty = {orbit.eccentricity}"
        string += f"\nInclination = {math.degrees(orbit.inclination)}"
        string += f"\nPeriod = {self.convert_time(orbit.period)}"

        logger.info(string)


if __name__ == "__main__":
    logging.basicConfig(format="%(asctime)-15s %(levelname)s %(message)s")
    logger = logging.getLogger(__name__)
    logger.setLevel(logging.DEBUG)

    conn = krpc.connect(name="Test Code")
    # Create the utility package
    utils = KrpcUtilities(conn)
    for inc in range(0, 360, 30):
        head = utils.launch_heading(inc, 100000)
        logger.info("Data: %s, %s", inc, head)
