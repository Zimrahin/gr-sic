/* -*- c++ -*- */
/*
 * Author: Diego Badillo-San-Juan <diego.badillo-san-juan@inria.fr>
 * Copyright 2025 Inria.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_BLE_BLE_PACKET_SINK_IMPL_H
#define INCLUDED_BLE_BLE_PACKET_SINK_IMPL_H

#include <gnuradio/ble/ble_packet_sink.h>

namespace gr {
namespace ble {

class ble_packet_sink_impl : public ble_packet_sink
{
private:
    template <typename T>
    T reverse_bits(T unsigned_integer); // Reverse bits in an unsigned integer
    uint64_t generate_access_code(uint32_t base_address); // Generate access code
    uint8_t slice(float data_in); // Slice float data into binary data

    uint d_access_code_len = 48; // 1B preamble + 4B base address + 1B address prefix
    uint64_t d_shift_reg = 0;    // Shift register for the incoming data
    uint d_fill_buffer = 0;      // Ensures the shift register is filled before comparing
    uint32_t d_base_address;     // Base address for the access code
    uint64_t d_access_code;      // Access code to be detected
    uint64_t d_mask;             // (1 << code_len) - 1
    uint d_threshold;            // Allowed bit errors in the preamble detection
    uint8_t d_lfsr;              // Linear Feedback Shift Register for whitening

public:
    ble_packet_sink_impl(uint32_t base_address, uint preamble_threshold, uint8_t lfsr);
    ~ble_packet_sink_impl();
    uint8_t whiten_bit(uint8_t data_bit,
                       uint8_t polynomial); // Whiten a single bit using LFSR

    // Called for each chunk of data in the input stream
    int work(int noutput_items,
             gr_vector_const_void_star& input_items,
             gr_vector_void_star& output_items);
};

} // namespace ble
} // namespace gr

#endif // INCLUDED_BLE_BLE_PACKET_SINK_IMPL_H
