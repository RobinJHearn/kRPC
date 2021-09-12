import krpc

print("Connecting")
conn = krpc.connect(name="Hello World")
print("Connected")
vessel = conn.space_center.active_vessel
print("Got vessel")

print(f"{vessel.name}")
