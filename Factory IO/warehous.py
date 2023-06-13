from pyModbusTCP.client import ModbusClient
c = ModbusClient(host="localhost", port=502, unit_id=1, auto_open=True)
c.open()

if c.read_discrete_inputs(0) == [True]: # At Tray Sensor
    c.write_single_coil(0,1) # Emitter Enabled

if c.read_discrete_inputs(1) ==[True]: # At Load Sensor
    c.write_multiple_coils(1,1) # Entry Conveyor on
    c.write_single_coil(2,1)
if c.read_discrete_inputs(1)==[False]: # At loadsensor is False
    c.write_multiple_coils(1,0) # Entry Conveyor on
    c.write_single_coil(2,0)
    c.write_single_coil(3,1) # Lift coil is on
    c.write_single_coil(3,1) # At left is on
    c.write_single_coil(3,1) # At middle is on