from pymavlink import mavutil

connection = mavutil.mavlink_connection(
    "/dev/ttyAMA2",
    baud=921600,
    source_system=255,
    source_component=190
)

print("Waiting for Pixhawk heartbeat...")
heartbeat = connection.wait_heartbeat(timeout=15)

if heartbeat is None:
    raise SystemExit("FAIL: No heartbeat received")

print("Heartbeat received")
print("System ID:", connection.target_system)
print("Component ID:", connection.target_component)

print("Requesting SYS_STATUS via MAV_CMD_REQUEST_MESSAGE...")
connection.mav.command_long_send(
    connection.target_system,
    connection.target_component,
    mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,
    0,
    mavutil.mavlink.MAVLINK_MSG_ID_SYS_STATUS,
    0, 0, 0, 0, 0, 0
)

message = connection.recv_match(type="COMMAND_ACK", blocking=True, timeout=5)
if message is None:
    print("No COMMAND_ACK received within 5 seconds")
else:
    print("COMMAND_ACK:", message)

print("Requesting param SERIAL2_BAUD...")
connection.mav.param_request_read_send(
    connection.target_system,
    connection.target_component,
    b"SERIAL2_BAUD",
    -1
)
param = connection.recv_match(type="PARAM_VALUE", blocking=True, timeout=5)
print("PARAM_VALUE:", param)

print("Diagnostic finished; no arming or movement command sent.")
