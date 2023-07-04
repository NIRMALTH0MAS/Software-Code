from pyModbusTCP.client import ModbusClient
c = ModbusClient(host="localhost", port=502, unit_id=1, auto_open=True)
c.open()

# Initial Condition all off
def initial_condition():
    if  c.read_discrete_inputs(0) == [True]: # At Tray Sensor
        c.write_single_coil(0,0) # Emitter disabled
        c.write_single_coil(1,0) # Entry Conveyor disabled
        c. write_single_coil(2,0) # Load Conveyor disabled
        return True
    else :
        return False

# Parcel Load Consition
def parcel_load():
    if c.read_discrete_inputs(0) == [True] and c.read_discrete_inputs(1) ==[True]: # At Load Sensor
        c.write_single_coil(0,1) # Emitter enabled
        while c.read_discrete_inputs(1) == [True]:
            c.write_single_coil(1,1) # Load Conveyor on
            c.write_single_coil(2,1) # Entry Conveyor on
        c.write_single_coil(0,0) # Emitter disabled
        c.write_single_coil(1,0) # Load Conveyor disabled
        c.write_single_coil(2,0) # Entry Conveyor disabled
        return True
    else:
        c.write_single_coil(0,0) # Emitter disabled
        c.write_single_coil(1,0) # Load Conveyor disabled
        c.write_single_coil(2,0) # Entry Conveyor disabled
        return False

if c.read_discrete_inputs(1)==[False]: # At loadsensor is False
    c.write_single_coil(1,0) # Entry Conveyor on
    c.write_single_coil(2,0)
    c.write_single_coil(3,1) # Lift coil is on
    c.write_single_coil(4,1) # At left is on
    c.write_single_coil(3,1) # At middle is on