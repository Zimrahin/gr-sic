/* -*- c++ -*- */
/*
 * Author: Diego Badillo-San-Juan <diego.badillo-san-juan@inria.fr>
 * Copyright 2025 Inria.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_SIC_BLE_PACKET_SINK_IMPL_H
#define INCLUDED_SIC_BLE_PACKET_SINK_IMPL_H

#include <gnuradio/sic/ble_packet_sink.h>

namespace gr {
namespace sic {

class ble_packet_sink_impl : public ble_packet_sink
{
public:
    ble_packet_sink_impl(uint32_t base_address,
                         uint preamble_threshold,
                         uint8_t lfsr,
                         uint block_id);
    ~ble_packet_sink_impl();

    // Called for each chunk of data in the input stream
    int work(int noutput_items,
             gr_vector_const_void_star& input_items,
             gr_vector_void_star& output_items);

private:
    // Finite State Machine
    enum class state { SEARCH_PREAMBLE, DECODE_LENGTH, DECODE_PAYLOAD, CHECK_CRC };
    state d_state;
    void enter_search_preamble();
    void enter_decode_length();
    void enter_decode_payload();
    void enter_check_crc();
    void process_search_preamble(uint8_t bit, uint64_t sample_index);
    void process_decode_length(uint8_t bit, uint64_t sample_index);
    void process_decode_payload(uint8_t bit, uint64_t sample_index);
    void process_check_crc(uint8_t bit, uint64_t sample_index);

    // Helper functions
    template <typename T>
    T reverse_bits(T unsigned_integer); // Reverse bits in an unsigned integer
    uint64_t generate_access_code(uint32_t base_address); // Generate access code
    uint8_t slice(float data_in); // Slice float data into binary data
    uint8_t whiten_bit(uint8_t data_bit,
                       uint8_t& lfsr,
                       uint8_t polynomial); // Whiten a single bit
    void compute_crc(uint8_t data_bit,
                     uint32_t& crc,
                     uint32_t polynomial,
                     uint32_t mask); // Compute CRC

    // Constants
    static const uint d_max_payload_len = 256;
    static const uint d_crc_len = 3;
    uint d_threshold;               // Allowed bit errors in the preamble detection
    uint d_access_code_len;         // 1B preamble + 4B base address + 1B address prefix
    uint32_t d_base_address;        // Base address for the access code
    uint64_t d_access_code;         // Access code to be detected
    uint64_t d_code_len_mask;       // (1 << code_len) - 1
    uint8_t d_lfsr_default;         // Linear Feedback Shift Register for whitening
    uint8_t d_whitening_polynomial; // Polynomial for whitening
    uint32_t d_crc_mask;            // Mask for CRC computation
    uint32_t d_crc_polynomial;      // Polynomial for CRC computation
    uint32_t d_crc_init;            // Initial value for CRC computation
    uint8_t d_num_preamble_bytes;   // Number of bytes after access code (S0 | LENGTH)
    uint d_block_id;                // Block instance ID

    // Variables
    bool d_output_connected;  // Indicates if the stream output port is connected
    uint64_t d_shift_reg;     // 64-bit shift register
    uint d_fill_buffer_count; // Ensures the shift register is filled before comparing
    uint8_t d_lfsr;           // Linear Feedback Shift Register for whitening
    uint8_t d_reg_byte;       // Shift register for byte decoding
    uint8_t d_bits_count;     // Number of bits collected for the current byte
    uint8_t d_bytes_count;    // Number of bytes collected for the current packet
    uint8_t d_payload_len;    // Length of the payload
    uint32_t d_crc;           // Computed CRC from payload and header
    std::array<uint8_t, d_max_payload_len> d_payload; // Payload buffer
    uint64_t d_sample_payload_index;                  // First sample index of the payload
    uint64_t d_packet_count; // Packet count for the current packet
    bool d_entering_payload; // Indicates if we are entering the payload state
};

} // namespace sic
} // namespace gr

#endif /* INCLUDED_SIC_BLE_PACKET_SINK_IMPL_H */
