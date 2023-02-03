from pymodbus.client import ModbusSerialClient


client = ModbusSerialClient(port='/COM8', timeout=1, baudrate=9600, bytesize=8, parity='N', stopbits=2)
client.connect()


while True:
    loadcell_res = client.read_holding_registers(address=100, count=1, slave=1)
    print(f"load Cell Value : {loadcell_res.registers[0]}")