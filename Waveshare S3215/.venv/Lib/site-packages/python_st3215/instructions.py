from enum import IntEnum


class Instruction(IntEnum):
    PING = 0x01
    READ = 0x02
    WRITE = 0x03
    REG_WRITE = 0x04
    ACTION = 0x05
    SYNC_READ = 0x82
    SYNC_WRITE = 0x83
    RESET = 0x06

    @classmethod
    def has_value(cls, value: int) -> bool:
        return value in cls._value2member_map_
