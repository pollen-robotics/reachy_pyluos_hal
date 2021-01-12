import struct
import numpy as np


class DxlModel:
    MX106 = 320
    MX64 = 310
    MX28 = 29
    AX18 = 18


def pos_from_raw(raw, model, offset, direct) -> float:
    if model in (DxlModel.MX106, DxlModel.MX64, DxlModel.MX28):
        max_deg, max_pos = 360.0, 4096
    else:
        max_deg, max_pos = 300.0, 1024

    pos = ((max_deg * raw) / (max_pos - 1)) - (max_deg / 2)
    pos = (pos if direct else -pos) - offset
    return np.deg2rad(pos)


def pos_to_raw(pos, model, offset, direct) -> int:
    pos = np.rad2deg(pos)
    pos = (pos + offset) * (1 if direct else -1)

    if model in (DxlModel.MX106, DxlModel.MX64, DxlModel.MX28):
        max_deg, max_pos = 360.0, 4096
    else:
        max_deg, max_pos = 300.0, 1024

    raw = int(round((max_pos - 1) * ((max_deg / 2 + pos) / max_deg), 0))
    raw = min(max(raw, 0), max_pos - 1)
    return raw


def load_from_raw(raw, model):
    return raw / 10.23


def load_to_raw(load, model):
    return int(load * 10.23)


def any_from_raw(value, model):
    return float(value)


def any_to_raw(value, model) -> int:
    return int(value)


def as_bytes(value, nb_bytes):
    if nb_bytes == 1:
        return [value]
    elif nb_bytes == 2:
        return list(struct.pack('H', value))
    else:
        return list(struct.pack('B' * nb_bytes, value))
