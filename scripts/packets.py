import time
from enum import IntFlag
from abc import ABC, abstractmethod
from pySerialTransfer.pySerialTransfer import SerialTransfer, Status, STRUCT_FORMAT_LENGTHS

class PacketID(IntFlag):
    SENSOR  = 0x01
    WATCHDOG = 0x02

class Packet(ABC):
    @abstractmethod
    def serialize(self, link):
        pass

    @abstractmethod
    def deserialize(self, link):
        pass

    def to_str(self):
        return str(self.__dict__)

class SensorPacket(Packet):
    def __init__(self):
        super().__init__()
        self.timestamp_ms = int(0)
        self.time_of_flight_ms = int(0)
        self.velocity_meters_per_second = float(0.0)

    def serialize(self, link):
        sendSize = 0
        sendSize = link.tx_obj(self.timestamp_ms, start_pos=sendSize, val_type_override='L')
        sendSize = link.tx_obj(self.time_of_flight_ms, start_pos=sendSize, val_type_override='L')
        sendSize = link.tx_obj(self.velocity_meters_per_second, start_pos=sendSize, val_type_override='f')
        return sendSize

    def deserialize(self, link):
        recSize = 0
        self.timestamp_ms = link.rx_obj(obj_type='L', start_pos=recSize)
        recSize += STRUCT_FORMAT_LENGTHS['L']
        self.time_of_flight_ms = link.rx_obj(obj_type='L', start_pos=recSize)
        recSize += STRUCT_FORMAT_LENGTHS['L']
        self.velocity_meters_per_second = link.rx_obj(obj_type='f', start_pos=recSize)

class WatchdogPacket(Packet):
    def __init__(self):
        super().__init__()
        self.overrun = bool(0)
        self.loop_time_ms = int(0)

    def serialize(self, link):
        sendSize = 0
        sendSize = link.tx_obj(self.overrun, start_pos=sendSize, val_type_override='B')
        sendSize = link.tx_obj(self.loop_time_ms, start_pos=sendSize, val_type_override='L')
        return sendSize

    def deserialize(self, link):
        recSize = 0
        self.overrun = bool(link.rx_obj(obj_type='B', start_pos=recSize))
        recSize += STRUCT_FORMAT_LENGTHS['B']
        self.loop_time_ms = link.rx_obj(obj_type='L', start_pos=recSize)


class SerialBridge:
    def __init__(self, port, baud, debug=False):
        self.link = SerialTransfer(port, baud)
        self.debug = debug

        self.link.open()
        time.sleep(2) # allow some time for the Arduino to completely reset

    def getTransfer(self):
        return self.link

    def send(self, pkt: Packet):
        '''
        Helper function to send a control packet to the Arduino. The control packet
        is defined in the packets.h file on the Arduino and is used to send commands
        or requests from the Python script to the Arduino.
        
        Parameters:
            pkt (Packet): The packet to send.
        '''

        if type(pkt) == SensorPacket:
            packet_id = PacketID.SENSOR
        elif type(pkt) == WatchdogPacket:
            packet_id = PacketID.WATCHDOG
        else:
            packet_id = 0   # Default to 0
        # Serialize and send packet
        send_size = pkt.serialize(self.link)
        self.link.send(send_size, packet_id)

    def receive(self, pkt=None):
        '''
        Helper function to receive a packet from the Arduino. The packet type can be
        specified to determine how to parse the incoming data.

        Returns:
            An instance of the received packet type with the parsed data, or None if no packet is available.
        '''
        if self.link.available():    # reads a packet

            if self.link.status.value < 0:
                if self.link.status == Status.CRC_ERROR:
                    print('ERROR: CRC_ERROR')
                elif self.link.status == Status.PAYLOAD_ERROR:
                    print('ERROR: PAYLOAD_ERROR')
                elif self.link.status == Status.STOP_BYTE_ERROR:
                    print('ERROR: STOP_BYTE_ERROR')
                else:
                    print(f'ERROR: {self.link.status.name}')

            if self.link.id_byte == PacketID.SENSOR:
                pkt = SensorPacket()
                pkt.deserialize(self.link)
                if self.debug:
                    print(f"Received Sensor Packet: {pkt.to_str()}")
                return pkt
            elif self.link.id_byte == PacketID.WATCHDOG:
                pkt = WatchdogPacket()
                pkt.deserialize(self.link)
                print(f"Received Watchdog Packet: {pkt.to_str()}")
                return pkt
