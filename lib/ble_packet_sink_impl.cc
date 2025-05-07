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

ble_packet_sink::sptr ble_packet_sink::make(uint32_t base_address,
                                            uint preamble_threshold)
{
    return gnuradio::make_block_sptr<ble_packet_sink_impl>(base_address,
                                                           preamble_threshold);
}

// Constructor
ble_packet_sink_impl::ble_packet_sink_impl(uint32_t base_address, uint preamble_threshold)
    : gr::sync_block("ble_packet_sink",
                     gr::io_signature::make(1, 1, sizeof(input_type)),
                     gr::io_signature::make(0, 0, 0)), // No output
      d_access_code(static_cast<uint64_t>(base_address)),
      d_mask((~uint64_t(0)) >> (64 - d_access_code_len)),
      d_threshold(preamble_threshold)
{
}

// Destructor
ble_packet_sink_impl::~ble_packet_sink_impl() {}

uint8_t ble_packet_sink_impl::slice(float data_in) { return data_in > 0 ? 1 : 0; }

int ble_packet_sink_impl::work(int noutput_items,
                               gr_vector_const_void_star& input_items,
                               gr_vector_void_star& output_items)
{
    auto in = static_cast<const input_type*>(input_items[0]);

    uint64_t sample_count = nitems_read(0);

    for (int i = 0; i < noutput_items; i++) {
        // Compute hamming distance between desired access code and current data
        uint64_t wrong_bits = 0;
        uint64_t nwrong = d_threshold + 1; // Value greater than the threshold

        if (d_fill_buffer < d_access_code_len) { // Buffer yet to be filled
            d_fill_buffer++;
        } else {
            wrong_bits = (d_shift_reg ^ d_access_code) & d_mask;
            volk_64u_popcnt(&nwrong, wrong_bits); // Count wrong bits

            // Debug print in hex
            std::cout << std::hex;
            // std::cout << "d_shift_reg = 0x" << d_shift_reg << ", ";
            // std::cout << "d_access_code = 0x" << d_access_code << ", ";
            std::cout << "d_mask = 0x" << d_mask << std::endl;
            // std::cout << "wrong_bits = 0x" << wrong_bits << std::dec << std::endl;

            // std::cout << "nwrong=" << nwrong << std::endl;
        }

        // Shift in new data
        d_shift_reg = (d_shift_reg << 1) | (slice(in[i]) & 1);
        if (nwrong <= d_threshold) {
            std::cout << "Detected access code at " << sample_count + i << std::endl;
        }
    }

    return noutput_items;
}


} /* namespace ble */
} /* namespace gr */
