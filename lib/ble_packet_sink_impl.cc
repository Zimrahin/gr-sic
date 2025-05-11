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
    d_mask = (~uint64_t(0)) >> (64 - d_access_code_len);
    d_access_code = generate_access_code(base_address);

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
uint8_t ble_packet_sink_impl::whiten_bit(uint8_t data_bit, uint8_t polynomial = 0x11)
{
    bool lfsr_msb = (d_lfsr & 0x40) != 0; // Bit 6 of the LFSR
    bool whitened_bit = data_bit ^ lfsr_msb;

    d_lfsr = static_cast<uint8_t>((d_lfsr << 1) & 0x7F);
    if (lfsr_msb)
        // Apply feedback
        d_lfsr ^= polynomial;

    return whitened_bit;
}

// Finite State Machine methods
void ble_packet_sink_impl::enter_search_preamble()
{
    d_shift_reg = 0;
    d_fill_buffer = 0;
    d_state = state::SEARCH_PREAMBLE;
}
void ble_packet_sink_impl::enter_decode_length()
{
    d_lfsr = d_lfsr_default; // Reset LFSR for whitening
    d_state = state::DECODE_LENGTH;
}
void ble_packet_sink_impl::process_search_preamble(uint8_t bit, uint64_t sample_index)
{
    // Buffer yet to be filled
    if (d_fill_buffer < d_access_code_len) {
        d_fill_buffer++;
        return;
    }
    // Count wrong bits
    uint64_t diff = (d_shift_reg ^ d_access_code) & d_mask;
    uint64_t nwrong = d_threshold + 1; // Value greater than the threshold
    volk_64u_popcnt(&nwrong, diff);

    if (nwrong <= d_threshold) {
        std::cout << "Preamble found at sample " << sample_index << "\n";
        enter_decode_length();
    }
}
void ble_packet_sink_impl::process_decode_length(uint8_t bit, uint64_t sample_index)
{
    // TODO: implement length decoding
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
        d_shift_reg = (d_shift_reg << 1) | (rx_bit & 1); // Shift in new bit

        switch (d_state) {
        case state::SEARCH_PREAMBLE:
            process_search_preamble(rx_bit, sample_count + i);
            break;

        case state::DECODE_LENGTH:
            process_decode_length(rx_bit, sample_count + i);
            break;
        }
    }
    return noutput_items;
}


} // namespace ble
} // namespace gr
