/* -*- c++ -*- */
/*
 * Author: Diego Badillo-San-Juan <diego.badillo-san-juan@inria.fr>
 * Copyright 2025 Inria.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */


#include "ble_packet_sink_impl.h"
#include <gnuradio/io_signature.h>
#include <volk/volk.h>
#include <iomanip>
#include <sstream>


namespace gr {
namespace ble {

using input_type = float;

ble_packet_sink::sptr
ble_packet_sink::make(uint32_t base_address, uint preamble_threshold, uint8_t lfsr)
{
    return gnuradio::make_block_sptr<ble_packet_sink_impl>(
        base_address, preamble_threshold, lfsr);
}

// Constructor
ble_packet_sink_impl::ble_packet_sink_impl(uint32_t base_address,
                                           uint preamble_threshold,
                                           uint8_t lfsr)
    : gr::sync_block("ble_packet_sink",
                     gr::io_signature::make(1, 1, sizeof(input_type)),
                     gr::io_signature::make(0, 0, 0)), // No output
      d_threshold(preamble_threshold),
      d_lfsr_default(lfsr)
{
    // Constants
    d_access_code_len = 48; // 1B preamble + 4B base address + 1B address prefix
    d_code_len_mask = (~uint64_t(0)) >> (64 - d_access_code_len);
    d_access_code = generate_access_code(base_address);
    d_whitening_polynomial = 0x11;
    d_crc_polynomial = 0x00065B;
    d_crc_mask = (~uint64_t(0)) >> (64 - CRC_LEN * 8);
    d_crc_init = 0x00FFFF;

    // Variables
    d_state = state::SEARCH_PREAMBLE;
}

// Destructor
ble_packet_sink_impl::~ble_packet_sink_impl() {}

// Slice float data into binary data
uint8_t ble_packet_sink_impl::slice(float data_in) { return data_in > 0 ? 1 : 0; }

// Reverse bits in an unsigned integer
template <typename T>
T ble_packet_sink_impl::reverse_bits(T unsigned_integer)
{
    static_assert(std::is_unsigned_v<T>,
                  "reverse_bits requires an unsigned integer type");
    T reversed = 0;
    constexpr int num_bits = std::numeric_limits<T>::digits;

    for (int i = 0; i < num_bits; ++i) {
        reversed <<= 1;
        reversed |= (unsigned_integer & 1);
        unsigned_integer >>= 1;
    }
    return reversed;
}

// Generate access code for packet detection
uint64_t ble_packet_sink_impl::generate_access_code(uint32_t base_address)
{
    uint64_t access_code = 0;
    uint8_t address_prefix = 0x00;
    uint8_t preamble = (base_address & 0x01) ? 0xAA : 0x55;
    base_address = reverse_bits(base_address);

    // From LSB to MSB: 0x00 | base_address | preamble
    access_code |= static_cast<uint64_t>(base_address) << 8 | address_prefix;
    access_code |= static_cast<uint64_t>(preamble) << 40;

    return access_code;
}

// Whiten a single bit using a 7‑bit LFSR
uint8_t
ble_packet_sink_impl::whiten_bit(uint8_t data_bit, uint8_t& lfsr, uint8_t polynomial)
{
    bool lfsr_msb = (d_lfsr & 0x40) != 0; // Bit 6 of the LFSR
    uint8_t whitened_bit = data_bit ^ lfsr_msb;

    lfsr = static_cast<uint8_t>((lfsr << 1) & 0x7F);
    if (lfsr_msb) {
        lfsr ^= polynomial; // Apply feedback
    }
    return whitened_bit & 1;
}

// Compute CRC from payload and header to compare with the received CRC
void ble_packet_sink_impl::compute_crc(uint8_t data_bit,
                                       uint32_t& crc,
                                       uint32_t polynomial,
                                       uint32_t mask)
{
    uint crc_len_bits = 8 * CRC_LEN;
    crc ^= data_bit << (crc_len_bits - 1);
    if (crc & (1 << (crc_len_bits - 1))) {
        crc = ((crc << 1) ^ polynomial) & mask; // Apply feedback
    } else {
        crc = (crc << 1) & mask;
    }
}


// Finite State Machine methods
void ble_packet_sink_impl::enter_search_preamble()
{
    d_shift_reg = 0;
    d_fill_buffer_count = 0;
    d_state = state::SEARCH_PREAMBLE;
}
void ble_packet_sink_impl::enter_decode_length()
{
    d_lfsr = d_lfsr_default;         // Reset LFSR for whitening
    d_crc = d_crc_init & d_crc_mask; // Compute CRC from the header
    d_payload_len = 0;
    d_reg_byte = 0;
    d_bits_count = 0;
    d_bytes_count = 0;
    d_state = state::DECODE_LENGTH;
}
void ble_packet_sink_impl::enter_decode_payload()
{
    d_reg_byte = 0;
    d_bits_count = 0;
    d_bytes_count = 0;
    d_state = state::DECODE_PAYLOAD;
}
void ble_packet_sink_impl::enter_check_crc()
{
    d_shift_reg = 0;
    d_fill_buffer_count = 0;
    d_state = state::CHECK_CRC;
}
void ble_packet_sink_impl::process_search_preamble(uint8_t bit, uint64_t sample_index)
{
    d_shift_reg = (d_shift_reg << 1) | bit; // Shift in new bit

    // Buffer yet to be filled
    if (d_fill_buffer_count < d_access_code_len - 1) {
        d_fill_buffer_count++;
        return;
    }
    // Count wrong bits
    uint64_t diff = (d_shift_reg ^ d_access_code) & d_code_len_mask;
    uint64_t nwrong = d_threshold + 1; // Value greater than the threshold
    volk_64u_popcnt(&nwrong, diff);

    if (nwrong <= d_threshold) {
        std::cout << std::dec << "Preamble found at sample " << sample_index << "\n";
        enter_decode_length();
    }
}
void ble_packet_sink_impl::process_decode_length(uint8_t bit, uint64_t sample_index)
{
    (void)sample_index;
    uint8_t num_bytes = 2; // Assume S0 | LENGTH, received from an nRF52
    uint8_t unwhitened_bit = whiten_bit(bit, d_lfsr, d_whitening_polynomial);
    compute_crc(unwhitened_bit, d_crc, d_crc_polynomial, d_crc_mask);

    d_reg_byte = (d_reg_byte >> 1) | (unwhitened_bit << 7); // Shift in new bit from MSB

    if (++d_bits_count < 8) {
        return; // Still collecting bits to unpack a byte
    }
    d_bits_count = 0; // Reset bit counter
    if (++d_bytes_count < num_bytes) {
        return; // We are only interested in the LENGTH byte
    }
    d_payload_len = d_reg_byte; // Unpack the LENGTH byte
    std::cout << std::dec << "Payload length: " << (int)d_payload_len << "\n";
    enter_decode_payload();
}
void ble_packet_sink_impl::process_decode_payload(uint8_t bit, uint64_t sample_index)
{
    (void)sample_index;
    uint8_t unwhitened_bit = whiten_bit(bit, d_lfsr, d_whitening_polynomial);
    compute_crc(unwhitened_bit, d_crc, d_crc_polynomial, d_crc_mask);

    d_reg_byte = (d_reg_byte >> 1) | (unwhitened_bit << 7); // Shift in new bit from MSB

    if (++d_bits_count < 8) {
        return; // Still collecting bits to unpack a byte
    }
    d_bits_count = 0; // Reset bit counter
    if (d_bytes_count < d_payload_len) {
        d_payload[d_bytes_count] = d_reg_byte;
        // std::cout << "0x" << std::hex << std::setfill('0') << std::setw(2)
        //           << static_cast<int>(d_payload[d_bytes_count]) << std::endl;

        if (++d_bytes_count == d_payload_len) {
            enter_check_crc();
        }
    }
}
void ble_packet_sink_impl::process_check_crc(uint8_t bit, uint64_t sample_index)
{
    (void)sample_index;
    uint8_t unwhitened_bit = whiten_bit(bit, d_lfsr, d_whitening_polynomial);
    d_shift_reg = (d_shift_reg << 1) | unwhitened_bit; // Shift in new bit

    // Buffer yet to be filled
    if (++d_fill_buffer_count < CRC_LEN * 8) {
        return;
    }
    if (d_fill_buffer_count == CRC_LEN * 8) {
        d_fill_buffer_count++;
        // Compare read CRC with computed CRC
        std::cout << "CRC, read: " << std::hex << std::setfill('0') << std::setw(6)
                  << d_shift_reg << std::endl;
        std::cout << "CRC, computed: " << std::hex << std::setfill('0') << std::setw(6)
                  << d_crc << std::endl;

        if (d_shift_reg == d_crc) {
            std::cout << "CRC check OK" << std::endl;
        } else {
            std::cout << "CRC check FAILED" << std::endl;
        }

        enter_search_preamble();
    }
}

// Called for each chunk of data in the input stream
int ble_packet_sink_impl::work(int noutput_items,
                               gr_vector_const_void_star& input_items,
                               gr_vector_void_star& output_items)
{
    auto in = static_cast<const input_type*>(input_items[0]);

    uint64_t sample_count = nitems_read(0);

    for (int i = 0; i < noutput_items; i++) {
        uint8_t rx_bit = slice(in[i]);

        switch (d_state) {
        case state::SEARCH_PREAMBLE:
            process_search_preamble(rx_bit, sample_count + i);
            break;

        case state::DECODE_LENGTH:
            process_decode_length(rx_bit, sample_count + i);
            break;

        case state::DECODE_PAYLOAD:
            process_decode_payload(rx_bit, sample_count + i);
            break;

        case state::CHECK_CRC:
            process_check_crc(rx_bit, sample_count + i);
            break;
        }
    }
    return noutput_items;
}


} // namespace ble
} // namespace gr
