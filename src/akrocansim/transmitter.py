import threading
import time
import can


_INDIVIDUAL_TX_MODE__TX_CONT = 0
_INDIVIDUAL_TX_MODE__TX_ONCE = 1
_TX_RATE_SEC = 2
_DATA = 3


class Transmitter:
    def __init__(self, J1939: dict):
        self._J1939 = J1939
        self.bus: can.BusABC = None
        self.PGNs_pending_tx = {}

        self.global_tx_mode__stop = True
        self.global_tx_mode__cont = False
        self.global_tx_mode__per_pgn = False
        self.tx_CAN_IDs = {}  # {(CAN_ID: int, is_extended: bool): {
                              #     _INDIVIDUAL_TX_MODE__TX_CONT: bool,
                              #     _INDIVIDUAL_TX_MODE__TX_ONCE: bool,
                              #     _TX_RATE_SEC: #,
                              #     _DATA: list
                              # }

        self.J1939_CAN_IDs = {}  # {PGN: CAN_ID}

    def register_tx_PGN(self, *, pgn, priority, source_address, tx_rate_ms):
        can_id = priority << 26 | pgn << 8 | source_address
        self.J1939_CAN_IDs[pgn] = (can_id, True)
        tx_rate_sec = tx_rate_ms / 1000
        self.tx_CAN_IDs[(can_id, True)] = {
            _INDIVIDUAL_TX_MODE__TX_CONT: False,
            _INDIVIDUAL_TX_MODE__TX_ONCE: False,
            _TX_RATE_SEC: tx_rate_sec,
            _DATA: [0 for _ in range(self._J1939[pgn]['PGN Data Length'])]
        }
        threading.Thread(target=self.send_periodic, args=(can_id, True), daemon=True).start()

    def send_periodic(self, can_id, is_extended):
        signal_spec = self.tx_CAN_IDs[(can_id, is_extended)]

        while True:
            time.sleep(signal_spec[_TX_RATE_SEC])
            if self.global_tx_mode__cont or (self.global_tx_mode__per_pgn and signal_spec[_INDIVIDUAL_TX_MODE__TX_CONT]):
                # transmit can message with 'can_id' this cycle
                pass
            elif signal_spec[_INDIVIDUAL_TX_MODE__TX_ONCE]:
                # transmit can message with 'can_id' this cycle and stop
                signal_spec[_INDIVIDUAL_TX_MODE__TX_ONCE] = False
            else:
                # do not transmit can message with 'can_id' this cycle
                continue

            if self.bus is not None:
                try:
                    self.bus.send(can.Message(arbitration_id=can_id, is_extended_id=is_extended, data=signal_spec[_DATA]))
                except can.CanOperationError:
                    pass

    def set_tx_mode_stop(self, pgn=None):
        if pgn is None:  # global
            self.global_tx_mode__stop = True
            self.global_tx_mode__cont = False
            self.global_tx_mode__per_pgn = False
        else:  # individual
            self.tx_CAN_IDs[self.J1939_CAN_IDs[pgn]][_INDIVIDUAL_TX_MODE__TX_CONT] = False

    def set_tx_mode_continuous(self, pgn=None):
        if pgn is None:  # global
            self.global_tx_mode__stop = False
            self.global_tx_mode__cont = True
            self.global_tx_mode__per_pgn = False
        else:  # individual
            self.tx_CAN_IDs[self.J1939_CAN_IDs[pgn]][_INDIVIDUAL_TX_MODE__TX_CONT] = True

    def set_tx_mode_per_PGN(self):
        self.global_tx_mode__stop = False
        self.global_tx_mode__cont = False
        self.global_tx_mode__per_pgn = True

    def set_tx_once(self, pgn=None):
        if pgn is None:  # global
            for signal_spec in self.tx_CAN_IDs.values():
                signal_spec[_INDIVIDUAL_TX_MODE__TX_ONCE] = True
        else:  # individual
            self.tx_CAN_IDs[self.J1939_CAN_IDs[pgn]][_INDIVIDUAL_TX_MODE__TX_ONCE] = True

    def modify_pgn_tx_rate(self, pgn, tx_rate_ms):
        tx_rate_sec = tx_rate_ms / 1000
        self.tx_CAN_IDs[self.J1939_CAN_IDs[pgn]][_TX_RATE_SEC] = tx_rate_sec

    def modify_pgn_data(self, pgn: int, spn: int, raw_value: int):
        data = self.tx_CAN_IDs[self.J1939_CAN_IDs[pgn]][_DATA]
        spn_spec = self._J1939[pgn]['SPNs'][spn]
        start_byte = spn_spec['start_byte']

        match spn_spec['length_bits']:
            case 1 | 2 | 3 | 4 | 5 | 6 | 7:
                start_bit = spn_spec['start_bit']
                length_bits = spn_spec['length_bits']
                mask = 0x00
                for i in range(length_bits):
                    mask = mask | 1 << start_bit + i
                data[start_byte] = data[start_byte] & ~mask | raw_value << start_bit & mask
            case 8:
                data[start_byte] = raw_value
            case 16:
                data[start_byte] = raw_value & 0x00FF
                data[start_byte + 1] = raw_value >> 8
            case 32:
                data[start_byte] = raw_value & 0x0000_00FF
                data[start_byte + 1] = raw_value >> 8 & 0x00_00FF
                data[start_byte + 2] = raw_value >> 16 & 0x00FF
                data[start_byte + 3] = raw_value >> 24

        self.tx_CAN_IDs[self.J1939_CAN_IDs[pgn]][_DATA] = bytearray(data)
