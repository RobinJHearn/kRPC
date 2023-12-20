"""Class holding general utility functions"""
import time
import krpc
import krpcutils
from logger import Logger

print("Connecting")
conn = krpc.connect(name="Hello World", address="Galileo")  # address="192.168.1.120")
print("Connected")
vessel = conn.space_center.active_vessel  # pylint: disable=no-member
print("Got vessel")

print(f"{vessel.name}")

utils = krpcutils.KrpcUtilities(logger=Logger().get(), connection=conn)

utils.jettison_fairings()
time.sleep(1)
utils.extend_solar_panels()


for part in vessel.parts.all:
    print(part.name)
    for module in part.modules:
        print(f"  {module.name}")
        for event in module.events:
            print(f"    {event}")
