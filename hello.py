"""Class holding general utility functions"""
import time

import krpc
import krpcutils

print("Connecting")
conn = krpc.connect(name="Hello World")
print("Connected")
vessel = conn.space_center.active_vessel  # pylint: disable=no-member
print("Got vessel")

print(f"{vessel.name}")


utils = krpcutils.KrpcUtilities(conn)

utils.jettison_fairings()
time.sleep(1)
utils.extend_solar_panels()
