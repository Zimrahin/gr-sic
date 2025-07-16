#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# Author: Diego Badillo-San-Juan <diego.badillo-san-juan@inria.fr>
# Copyright 2025 Inria.
#
# SPDX-License-Identifier: GPL-3.0-or-later
#

import numpy as np


# Find a sequence of bits in a given binary dara array
def correlate_access_code(data: np.ndarray, access_code: str, threshold: int, reduce_mask: bool = False) -> np.ndarray:
    """Find a sequence of bits in a given binary dara array."""
    # access_code: from LSB to MSB (as samples arrive on-air)
    access_code = access_code.replace("_", "")
    code_len = len(access_code)
    access_int = int(access_code, 2)  # Convert the access code (e.g. "10110010") to an integer
    mask = (1 << code_len) - 1  # Create a mask to keep only the last code_len bits.
    if reduce_mask:
        # Create a mask that has ones everywhere except at the MSB and LSB:
        # This is useful for differential encoding in chip sequences for IEEE 802.15.4 demodulation
        reduced_mask = mask & ~((1 << (code_len - 1)) | 1)
    else:
        reduced_mask = mask
    data_reg = 0
    positions = []  # Positions where the access code has been found in data

    for i, bit in enumerate(data):
        bit = int(bit)
        data_reg = ((data_reg << 1) | (bit & 0x1)) & mask  # Shift in the new bit
        if i + 1 < code_len:  # Start comparing once data_reg is filled
            continue

        # Count the number of mismatched bits between data_reg and the access code
        mismatches = bin((data_reg ^ access_int) & reduced_mask).count("1")
        if mismatches <= threshold:
            # Report the position immediately after the access code was found
            positions.append(i + 1)

    return np.array(positions)


# Apply whintening (de-whitening) to an array of bytes
def ble_whitening(data: np.ndarray, lfsr=0x01, polynomial=0x11):
    """Apply whintening (de-whitening) to an array of bytes."""
    # LFSR default value is 0x01 as it is the default value in the nRF DATAWHITEIV register
    # The polynomial default value is 0x11 = 0b001_0001 -> x⁷ + x⁴ + 1 (x⁷ is omitted)
    output = np.empty_like(data)  # Initialise output array

    for idx, byte in enumerate(data):
        whitened_byte = 0
        for bit_pos in range(8):
            # XOR the current data bit with LFSR MSB
            lfsr_msb = (lfsr & 0x40) >> 6  # LFSR is 7-bit, so MSB is at position 6
            data_bit = (byte >> bit_pos) & 1  # Extract current bit
            whitened_bit = data_bit ^ lfsr_msb  # XOR

            whitened_byte |= whitened_bit << (bit_pos)  # Update the whitened byte

            # Update LFSR
            if lfsr_msb:  # If MSB is 1 (before shifting), apply feedback
                lfsr = (lfsr << 1) ^ polynomial  # XOR with predefined polynomial
            else:
                lfsr <<= 1
            lfsr &= 0x7F  # 0x7F mask to keep lsfr within 7 bits

        output[idx] = whitened_byte

    return output, lfsr


# Pack a sequence of bits (array) into an array of bytes (integers)
def pack_bits_to_uint8(bits: np.ndarray) -> np.ndarray:
    """Pack a sequence of bits (array) into an array of bytes (integers)."""
    # Pack binary array LSB first ([1,1,1,1,0,0,0,0]) into bytes array ([0x0F])
    # Ensure the binary array length is a multiple of 8
    if len(bits) % 8 != 0:
        raise ValueError(f"The binary list {len(bits)} length must be a multiple of 8.")

    # Convert binary to NumPy array
    bits = np.array(bits, dtype=np.uint8)
    bits = bits.reshape(-1, 8)[:, ::-1]  #  LSB to MSB correction
    uint8_array = np.packbits(bits, axis=1).flatten()

    return uint8_array


# Computes the Cyclic Redundancy Check for a given array of bytes
def compute_crc(
    data: np.ndarray,
    crc_init: int = 0x00FFFF,
    crc_poly: int = 0x00065B,
    crc_size: int = 3,
) -> np.ndarray:
    """Computes the Cyclic Redundancy Check for a given array of bytes."""
    crc_mask = (1 << (crc_size * 8)) - 1  # Mask to n-byte width (0xFFFF for crc_size = 2)

    def swap_nbit(num, n):
        num = num & crc_mask
        reversed_bits = f"{{:0{n * 8}b}}".format(num)[::-1]
        return int(reversed_bits, 2)

    crc_init = swap_nbit(crc_init, crc_size)  # LSB -> MSB
    crc_poly = swap_nbit(crc_poly, crc_size)

    crc = crc_init
    for byte in data:
        crc ^= int(byte)
        for _ in range(8):  # Process each bit
            if crc & 0x01:  # Check the LSB
                crc = (crc >> 1) ^ crc_poly
            else:
                crc >>= 1
            crc &= crc_mask  # Ensure CRC size

    return np.array([(crc >> (8 * i)) & 0xFF for i in range(crc_size)], dtype=np.uint8)


# Generates (preamble + base address sequence) to use as access code
def generate_access_code_ble(base_address: int) -> str:
    """Generates (preamble + base address sequence) to use as access code."""
    base_address &= 0xFFFFFFFF  # 4-byte unsigned long
    preamble = 0x55 if base_address & 0x01 else 0xAA
    preamble = format(preamble, "08b")[::-1]

    base_address = base_address.to_bytes(4, byteorder="little")
    base_address = [format(byte, "08b")[::-1] for byte in base_address]
    address_prefix = "00000000"

    return "_".join([preamble] + base_address + [address_prefix])


# Returns a string of chips to be used by correlate access code function
def map_nibbles_to_chips(
    byte_array: np.ndarray, chip_mapping: np.ndarray, return_string: bool = True
) -> str | np.ndarray:
    """Returns a string of chips to be used by correlate access code function."""
    result = []
    for byte in byte_array:
        for nibble in [byte & 0x0F, (byte >> 4) & 0x0F]:  # Extract LSB first, then MSB
            mapped_value = chip_mapping[nibble]
            if return_string:
                binary_string = f"{mapped_value:032b}"  # Convert to 32-bit binary
                result.append(binary_string)  # Append delimiter
            else:
                result.append(mapped_value)  # Append uint32 chip value

    return "_".join(result) if return_string else np.array(result, dtype=np.uint32)


# Return the number of set bits in the lowest 'bits' of 'n'.
def count_set_bits(n: int, bits: int = 32) -> int:
    """Return the number of set bits in the lowest 'bits' of 'n'."""
    mask = (1 << bits) - 1  # Create a mask for the lowest 'bits' bits
    return bin(n & mask).count("1")


# Decodes the received chip sequence by comparing it against a known mapping.
def decode_chips(chips32: int, chip_mapping: np.ndarray, threshold: int = 32) -> int:
    """Decodes the received chip sequence by comparing it against a known mapping."""
    best_match = 0xFF
    min_threshold = 33  # Value greater than the maximum possible errors (32 bits)

    for i in range(16):
        # 0x7FFFFFFE masks out the first and last bit, since these depend on previous chip data
        # This is because we are using differential encoding
        masked_diff = (chips32 ^ chip_mapping[i]) & 0x7FFFFFFE
        diff_bits = count_set_bits(masked_diff, 32)  # Count the number of bits that differ

        if diff_bits < min_threshold:
            best_match = i
            min_threshold = diff_bits

    if min_threshold <= threshold:
        return best_match & 0xF  # Return position in chip mapping

    return 0xFF  # If no valid match was found, return 0xFF to indicate an error


# Pack chips into bytes. Assumes each byte is formed from 64 chips (32 per nibble).
def pack_chips_to_bytes(chips: np.ndarray, num_bytes: int, chip_mapping: np.ndarray, threshold: int) -> np.ndarray:
    """Pack chips into bytes. Assumes each byte is formed from 64 chips (32 per nibble)."""
    bytes_out = np.empty(num_bytes, dtype=np.uint8)

    for i in range(num_bytes):
        # Convert 32 chips into an integer
        nibble1 = chips_to_int(chips[i * 64 : i * 64 + 32])
        nibble2 = chips_to_int(chips[i * 64 + 32 : (i + 1) * 64])

        # Decode the nibbles
        nibble1 = decode_chips(nibble1, chip_mapping=chip_mapping, threshold=threshold)
        nibble2 = decode_chips(nibble2, chip_mapping=chip_mapping, threshold=threshold)

        # Pack into a byte
        bytes_out[i] = nibble1 | (nibble2 << 4)

    return bytes_out


# Convert a 32-length binary array to an integer.
def chips_to_int(chips: np.ndarray) -> int:
    """Convert a 32-length binary array to an integer."""
    assert len(chips) == 32, "Input must be exactly 32 elements long"
    return int("".join(map(str, chips)), 2)


# Create a physical BLE packet from payload and base address.
def create_ble_phy_packet(payload: np.ndarray, base_address: int) -> np.ndarray:
    """
    Create a physical BLE packet from payload and base address.

    Packet Structure:
    ┌───────────┬──────────────┬───────────────┬───────────┬────────┬────────┬─────────┐
    │ Preamble  │ Base Address │ Prefix (0x00) │ S0 (0x00) │ Length │ PDU    │ CRC     │
    ├───────────┼──────────────┼───────────────┼───────────┼────────┼────────┼─────────┤
    │ 0xAA/0x55 │ 4 Bytes      │ 1 Byte        │ 1 Byte    │ 1 Byte │ 0-255B │ 3 Bytes │
    └───────────┴──────────────┴───────────────┴───────────┴────────┴────────┴─────────┘
    """
    max_payload_size: int = 255  # Bytes
    # Crop the payload if it exceeds the maximum allowed size
    if len(payload) > max_payload_size:
        payload = payload[:max_payload_size]
        print(
            f"Warning: create_ble_phy_packet() - Payload exceeded the maximum allowed size ({max_payload_size}B) and has been cropped."
        )

    # Set the preamble based on the LSB of the base address
    preamble = np.uint8(0x55 if (base_address & 0x01) else 0xAA)

    # Convert base address into 4 bytes in little-endian order
    base_addr_len = 4
    base_addr_bytes = np.array(
        [np.uint8((base_address >> (i * 8)) & 0xFF) for i in range(base_addr_len)],
        dtype=np.uint8,
    )

    # Set prefix, S0 and length bytes
    prefix = np.uint8(0x00)
    s0 = np.uint8(0x00)
    length = np.uint8(len(payload))

    # Append CRC
    ready_for_crc = np.concatenate(([s0, length], payload))
    crc = compute_crc(ready_for_crc, crc_init=0x00FFFF, crc_poly=0x00065B, crc_size=3)
    ready_for_whitening = np.concatenate((ready_for_crc, crc))

    # Whiten from S0 (included) to CRC (included)
    whitened, _ = ble_whitening(ready_for_whitening)
    packet = np.concatenate(([preamble], base_addr_bytes, [prefix], whitened))

    return packet


# Create a physical IEEE 802.15.4 packet from payload. CRC is optional.
def create_802154_phy_packet(payload: np.ndarray, append_crc: bool) -> np.ndarray:
    """
    Create a physical IEEE 802.15.4 packet from payload. CRC is optional.

    Packet Structure:
    ┌──────────────────────────────┬────────┬───────────────────────────┬────────────────┐
    │ Preamble sequence            │ Length │ PDU                       | CRC (optional) │
    ├──────────────────────────────┼────────┼───────────────────────────┼────────────────┤
    │ 0x00, 0x00, 0x00, 0x00, 0xA7 │ 1 Byte │ 0-125B/127B (CRC / no CRC)│ 2 Bytes        │
    └──────────────────────────────┴────────┴───────────────────────────┴────────────────┘
    """
    crc_size = 2
    max_payload_size = 127  # Bytes
    max_payload_size_with_crc = max_payload_size - crc_size  # Bytes
    preamble = np.array([0x00, 0x00, 0x00, 0x00, 0xA7], dtype=np.uint8)  # Set the preamble

    # Adjust payload size based on CRC inclusion
    if append_crc:
        if len(payload) > max_payload_size_with_crc:
            payload = payload[:max_payload_size_with_crc]
            print(
                f"Warning: create_802154_phy_packet() - Payload exceeded {max_payload_size_with_crc}B (CRC enabled) and has been cropped."
            )
        # Set length byte and CRC
        length = np.uint8(len(payload) + crc_size)
        crc = compute_crc(payload, crc_init=0x0000, crc_poly=0x011021, crc_size=crc_size)
        packet = np.concatenate((preamble, [length], payload, crc))  # Assemble packet

    else:
        if len(payload) > max_payload_size:
            payload = payload[:max_payload_size]
            print(f"Warning: create_802154_phy_packet() - Payload exceeded {max_payload_size}B and has been cropped.")
        length = np.uint8(len(payload))  # Set length byte
        packet = np.concatenate((preamble, [length], payload))  # Assemble packet

    return packet


# Unpack an array of bytes (np.uint8) into an array of bits, LSB first
def unpack_uint8_to_bits(uint8_array: np.ndarray) -> np.ndarray:
    """Unpack an array of bytes (np.uint8) into an array of bits, LSB first"""
    # Unpack bits from each byte into a matrix where each row is the binary representation
    bits = np.unpackbits(uint8_array).reshape(-1, 8)
    bits = bits[:, ::-1]  # LSB first as sent on air
    return bits.flatten()


# Maps a the even and odd chips in a uint32 input array to I chips and Q chips respectively.
def split_iq_chips(uint32_chips: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Maps a the even and odd chips in a uint32 input array to I chips and Q chips respectively."""
    # Preallocate output arrays
    total_bits = uint32_chips.size * 32
    I_chips = np.empty(total_bits // 2, dtype=np.uint8)
    Q_chips = np.empty(total_bits // 2, dtype=np.uint8)

    index = 0
    for value in uint32_chips:
        # Convert each uint32 to a numpy array of 1s and 0s
        bits = np.array([(value >> i) & 1 for i in range(31, -1, -1)], dtype=np.uint8)

        # Even-indexed chips go to I_chips, odd-indexed bits to Q_chips
        I_chips[index : index + 16] = bits[::2]
        Q_chips[index : index + 16] = bits[1::2]

        index += 16  # Increase return index for each value in uint32_array

    return I_chips, Q_chips


# Searches for an IEEE 802.15.4 preamble in hard decisions `chip_samples`.
def preamble_detection_802154(
    chip_samples: np.ndarray,
    threshold: int,
    chip_mapping: np.ndarray,
    pattern: np.ndarray = np.array([0x00, 0x00, 0x00, 0x00, 0xA7]),
) -> np.ndarray:
    """
    Searches for an IEEE 802.15.4 preamble in `chip_samples`.
    It first looks for 0x00 chip mappings as an a priori estimation, and then from each match,
    it searches for the rest of the preamble.
    """
    if len(pattern) == 0:
        return np.array([])  # No pattern means no detection

    # Detect potential preamble positions by searching for the first byte of the pattern
    access_code = map_nibbles_to_chips([pattern[0]], chip_mapping)
    preamble_positions = correlate_access_code(chip_samples, access_code, threshold=threshold, reduce_mask=True)

    if len(pattern) == 1:
        return np.array(preamble_positions)

    preamble_positions_final = []

    # Verify the remaining bytes of the pattern
    for position in preamble_positions:
        for byte in pattern[1:]:
            next_byte = pack_chips_to_bytes(
                chip_samples[position : position + 64],
                num_bytes=1,
                chip_mapping=chip_mapping,
                threshold=threshold,
            )
            if next_byte == byte:
                position += 64  # Move to the next byte position
            else:
                break
        else:
            # All iterations passed
            preamble_positions_final.append(position)

    return np.array(preamble_positions_final)


def triangular_wave(step: int, length: int) -> np.ndarray:
    """Used to generate a payload template"""
    max_val = np.iinfo(np.uint8).max
    ascend = np.arange(0, max_val + 1, step, dtype=np.uint8)
    descend = ascend[-2:0:-1]
    cycle = np.concatenate((ascend, descend))
    repeats = int(np.ceil(length / cycle.size))
    return np.tile(cycle, repeats)[:length]
