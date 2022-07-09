"""
Class of useful utility functions
"""

import logging
import math
import time

import krpc
import pid

from decorators import log_debug
from vector import Vector
from hillclimb import hill_climb

# Universal Constants
G0 = 9.80665
HOURS_PER_DAY = 6
SECONDS_PER_DEGREE = 60


class KrpcUtilities(object):
    """Some useful utility functions"""

    def __init__(self, logger, connection):
        self.conn = connection
        self.ksc = self.conn.space_center  # pylint: disable=no-member
        self.vessel = self.ksc.active_vessel
        self.logger = logger
        self.sas_mode = self.ksc.SASMode
        self.available_thrust = -100

    def do_prelaunch(self):
        """Pre-launch setup"""
        self.vessel.control.sas = True
        self.vessel.control.sas_mode = self.sas_mode.stability_assist
        self.vessel.control.rcs = False
        self.vessel.control.throttle = 1.0

    def do_launch(self):
        """Launch the rocket pointing straight up"""

        # Activate the first stage
        self.vessel.auto_pilot.engage()
        self.vessel.auto_pilot.target_pitch_and_heading(90, 90)
        # Stage until the rocket gets some thrust
        while self.vessel.thrust < 10:
            self.vessel.control.activate_next_stage()
            time.sleep(1)

    def do_ascent(
        self,
        target_altitude=100_000,
        target_inclination=0,
        perform_roll=False,
        full_burn=False,
    ):
        """Perform an ascent, pitching over as the rocket climbs.

        Parameters:
        `target_altitude`: the target altituide to reach
        `target_heading`: the heading to use when ascending
        """
        self.logger.info(
            "Ascending to apoapsis %s on heading %s",
            target_altitude,
            target_inclination,
        )

        # Set the initial parameters, heading, pitch and roll
        heading = self.launch_heading(target_inclination, target_altitude)
        self.logger.info("Target heading %s", heading)
        self.vessel.auto_pilot.target_heading = heading
        self.vessel.auto_pilot.target_pitch = 90
        if perform_roll:
            self.vessel.auto_pilot.target_roll = 0
        else:
            self.vessel.auto_pilot.target_roll = 90

        with self.conn.stream(
            getattr, self.vessel.orbit, "apoapsis_altitude"
        ) as apoapsis, self.conn.stream(
            getattr, self.vessel.flight(), "surface_altitude"
        ) as altitude, self.conn.stream(
            getattr, self.vessel.orbit, "time_to_apoapsis"
        ) as time_to_apoapsis:

            while apoapsis() < target_altitude * 0.95:

                # Get rid of any used stages
                self.auto_stage()

                # Adjust the pitch
                self.vessel.auto_pilot.target_pitch = max(
                    88.963 - 1.03287 * altitude() ** 0.409511, 0
                )

                self.logger.debug(
                    "Current Heading %s", self.vessel.auto_pilot.target_heading
                )

                if not full_burn:
                    # Apply a simple change to thrust based on time to apoapsis
                    if time_to_apoapsis() < 60:
                        self.vessel.control.throttle = 1.0
                    if time_to_apoapsis() > 65 and time_to_apoapsis() < 120:
                        self.vessel.control.throttle = 0.5
                    if time_to_apoapsis() > 125:
                        self.vessel.control.throttle = 0.25

            self.vessel.control.throttle = 0.1
            while apoapsis() < target_altitude:
                pass

        self.logger.info("Target apoapsis reached")
        self.vessel.control.throttle = 0.0

    def do_coast(self, target_altitude):
        """Coast until out of the atmosphere, keeping apoapsis at `altitude`

        Parameters:
        `altitude`: the target altitude to hold
        """
        self.logger.info(
            "Coasting out of atmosphere to apoapsis of %s", target_altitude
        )

        self.vessel.auto_pilot.target_pitch = 0

        coast_pid = pid.PID(P=0.3, I=0, D=0)
        coast_pid.setpoint(target_altitude)

        # Until we exit the planets atmosphere
        atmosphere_depth = self.vessel.orbit.body.atmosphere_depth
        with self.conn.stream(
            getattr, self.vessel.flight(), "mean_altitude"
        ) as altitude, self.conn.stream(
            getattr, self.vessel.orbit, "apoapsis_altitude"
        ) as apoapsis:
            while altitude() < atmosphere_depth:
                self.vessel.control.throttle = self.limit(
                    self.vessel.control.throttle + coast_pid.update(apoapsis())
                )
                self.auto_stage()
                time.sleep(0.1)

    #    @log_debug(logging.getLogger("KSP"))
    def auto_stage(self):
        """Stage engines when thrust drops"""
        if self.available_thrust < -10:
            self.available_thrust = self.vessel.available_thrust - 10
        self.logger.debug(
            "Avaiable Thrust: %s, Last Value: %s",
            self.vessel.available_thrust,
            self.available_thrust,
        )
        if self.vessel.available_thrust < self.available_thrust:
            while True:
                self.vessel.control.activate_next_stage()
                self.logger.info("Staging")
                if self.vessel.available_thrust > 0:
                    self.available_thrust = self.vessel.available_thrust - 10
                    break

    def jettison_fairings(self):
        """Jettison all fairings"""
        for fairing in self.vessel.parts.fairings:
            fairing.jettison()

    def extend_solar_panels(self):
        """Extend all solar panels"""
        for solar_panel in self.vessel.parts.solar_panels:
            if solar_panel.deployable:
                solar_panel.deployed = True

    def extend_antennas(self):
        """Extend all antennas"""
        for antenna in self.vessel.parts.antennas:
            if antenna.deployable:
                antenna.deployed = True

    def wait_until_time(self, ut_time, use_warp=True):
        """Wait until it is specified absolute time

        Parameters:
        `ut_time`: Absolute universal time in seconds to wait until
        """
        self.logger.info("Waiting until time")
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
            self.vessel.control.sas = False
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
        self.logger.info("Orientating ship for burn")
        if use_sas:
            self.vessel.auto_pilot.disengage()
            if self.set_sas_mode(self.sas_mode.maneuver):
                self.logger.info("Using SAS")
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
                self.logger.info("Returning to Auto Pilot")
            else:
                self.logger.info("Using auto pilot")

        error = 100
        while math.fabs(error) > 2:
            self.vessel.control.sas = False
            self.vessel.auto_pilot.sas = False
            self.vessel.auto_pilot.reference_frame = node.reference_frame
            self.vessel.auto_pilot.target_direction = (0, 1, 0)
            self.vessel.auto_pilot.engage()
            self.vessel.auto_pilot.wait()
            error = self.vessel.auto_pilot.error
            self.logger.info("Error: %s", error)

    def execute_burn(self, node):
        """Execute the burn defined by the node, assume the craft is already aligned for the burn

        Parameters:
        `node`: Node defining the deltaV needed for the burn
        """
        self.logger.info("Executing burn")

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
            self.logger.error("execute_next_node: No Node to execute")

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

        self.logger.debug(
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

        self.logger.debug("Launch Angle %s", launch_angle)

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
        self.logger.debug("Launch Heading %s", b_rot)

        return b_rot

    def countdown(self, seconds):
        """Display a simple countdown to launch

        Parameters:
        `seconds`: the number of ticks in the countdown
        """
        for tick in range(seconds, 0, -1):
            self.logger.info("%s...", tick)
            time.sleep(1)
        self.logger.info("Launch!")

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

        self.logger.info(string)

    def wait_for_LAN(self, target_lan):  # pylint: disable=invalid-name
        """Wait to launch at a specific LAN
        The body.rotation_angle is the LAN that would occur if the
        launch was from 0 longitude.

        The target rotation angle to launch at is
            target_lan - launch site longitude
        Subtract the current rotation angle gives teh number of
        degrees between 'now' and 'then' so convert to time by
        multiplying by 60 (360 degress = 6 hours, so 1 degree per minute)

        Launching at this time will produce a LAN near to what is requested but
        will overshoot as the rocket is thrown east on launch by the rotation
        of the planet.

        """

        if target_lan:
            self.logger.info("Waiting for LAN of %s", target_lan)

            target_rotation_angle = self.limit_absolute_angle(
                target_lan - self.vessel.flight().longitude
            )
            delta_angle = target_rotation_angle - math.degrees(
                self.vessel.orbit.body.rotation_angle
            )
            time_to_wait = delta_angle * SECONDS_PER_DEGREE
            self.logger.info(
                "Target: %s, Rotation: %s, Delta: %s, Time: %s",
                target_lan,
                target_rotation_angle,
                delta_angle,
                time_to_wait,
            )
            self.ksc.warp_to(self.ksc.ut + time_to_wait - 30)

            # Wait to get closer to the required LAN
            with self.conn.stream(
                getattr, self.vessel.orbit.body, "rotation_angle"
            ) as angle:
                delta = math.degrees(target_rotation_angle - angle())
                while delta > 0 and delta < -1:
                    time.sleep(1.0)

            self.logger.info(
                "Angle: %s", math.degrees(self.vessel.orbit.body.rotation_angle)
            )

    def do_test(self, test_altitude, test_speed):
        """Test a component at an altitude/speed"""

        logger.info("Target Alt: %s, Target Speed: %s", test_altitude, test_speed)
        srf_frame = self.vessel.orbit.body.reference_frame

        with conn.stream(
            getattr, self.vessel.flight(), "mean_altitude"
        ) as altitude, conn.stream(
            getattr, self.vessel.flight(srf_frame), "speed"
        ) as speed:
            while altitude() < test_altitude:
                logger.info("Altitude: %s, Speed: %s", altitude(), speed())
                if speed() > 1.1 * test_speed:
                    self.vessel.control.throttle = 0.0
                if speed() < 0.9 * test_speed:
                    self.vessel.control.throttle = 1.0
                time.sleep(1.0)

            # Run the test
            for part in self.vessel.parts.all:
                for module in part.modules:
                    if module.has_event("Run Test"):
                        module.trigger_event("Run Test")

    def score_inclination(self, target_inclination, target_time):
        """Wrapper to set a target inclination value"""

        @log_debug(logger)
        def score_function(data):
            """Scoring function for inclination changes

            Parameters:
            `data`: list of three items representing prograde, radial and normal burns
            """
            node = self.add_node([target_time, 0, 0, data[0]])
            inclination = math.degrees(node.orbit.inclination)
            logger.debug("target inc: %s node.inc: %s", target_inclination, inclination)
            score = abs(target_inclination - inclination)
            node.remove()
            return score

        return score_function

    def inclination_change(self, inclination):
        """Change inclination of orbit

        Inclination change only requires a normal burn (preferrably at an
        ascending or descending node)
        """
        logger.info("Changing orbit inclination to %s degrees", inclination)

        (an_time, dn_time) = self.time_ascending_descending_nodes(delta_time=False)
        # Which node do we want
        if inclination > math.degrees(self.vessel.orbit.inclination):
            # Going down so descending node
            node_time = dn_time
        else:
            node_time = an_time

        score_function = self.score_inclination(inclination, node_time)
        data = hill_climb([0], score_function)[0]
        self.add_node([node_time, 0, 0, data])


if __name__ == "__main__":
    logging.basicConfig(format="%(asctime)-15s %(levelname)s %(message)s")
    my_logger = logging.getLogger("KSP")
    my_logger.setLevel(logging.DEBUG)

    conn = krpc.connect(name="Test Code")
    # Create the utility package
    utils = KrpcUtilities(my_logger, conn)

    utils.jettison_fairings()
    time.sleep(2)
    utils.extend_solar_panels()
    utils.extend_antennas()
