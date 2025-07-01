/* -*- c++ -*- */
/*
 * Author: Diego Badillo-San-Juan <diego.badillo-san-juan@inria.fr>
 * Copyright 2025 Inria.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

/*
 * References:
 * - bastibl/gr-ieee802-15-4 (packet_sink.cc)
 * - GNU Radio 802.15.4 En- and Decoding
 * - CMOS RFIC Architectures for IEEE 802.15.4 Networks
 */

#include "ieee802154_packet_sink_impl.h"
#include <gnuradio/io_signature.h>
#include <volk/volk.h>

namespace gr {
namespace ble {

using input_type = float;
using output_type = uint8_t; // Optional output, sliced data

ieee802154_packet_sink::sptr
ieee802154_packet_sink::make(uint preamble_threshold, bool crc_included, uint block_id)
{
    return gnuradio::make_block_sptr<ieee802154_packet_sink_impl>(
        preamble_threshold, crc_included, block_id);
}

// Constructor
ieee802154_packet_sink_impl::ieee802154_packet_sink_impl(uint preamble_threshold,
                                                         uint block_id)
    : gr::sync_block("ieee802154_packet_sink",
                     gr::io_signature::make(1, 1, sizeof(input_type)),
                     gr::io_signature::make(0, 1, sizeof(output_type))),
      d_threshold(preamble_threshold),
      d_crc_included(crc_included),
      d_block_id(block_id)
{
    // Constants
    d_chip_mask =
        0x7FFFFFFE; // Ignore the first and last chips for differential MSK decoding

    // Variables
    d_packet_count = 0;

    // Enter first FSM state
    enter_search_preamble();

    // PMT message output port
    message_port_register_out(pmt::mp("pdu"));
}

// Destructor
ieee802154_packet_sink_impl::~ieee802154_packet_sink_impl() {}

// Slice float data into binary data
uint8_t ieee802154_packet_sink_impl::slice(float data_in) { return data_in > 0 ? 1 : 0; }

// Checks whether a given chip sequence matches the predefined constant channel mapping at
// index nibble, with a tolerance of threshold errors
bool ieee802154_packet_sink_impl::nibble_match(uint32_t chip_sequence,
                                               uint8_t nibble,
                                               uint threshold)
{
    uint32_t nwrong;
    uint32_t diff = (chip_sequence ^ d_chip_mapping_msk.at(nibble)) & d_chip_mask;
    volk_32u_popcnt(&nwrong, diff);

    return nwrong <= threshold;
}

// Pack 32 chips into a nibble
uint8_t ieee802154_packet_sink_impl::pack_chips_to_nibble(uint32_t chip_sequence,
                                                          uint threshold)
{
    uint8_t best_match = 0xFF;
    uint min_threshold = threshold;

    for (uint8_t nibble = 0; nibble < d_chip_mapping_msk.size(); nibble++) {
        uint32_t nwrong;
        uint32_t diff = (chip_sequence ^ d_chip_mapping_msk.at(nibble)) & d_chip_mask;
        volk_32u_popcnt(&nwrong, diff);

        if (nwrong <= min_threshold) {
            best_match = nibble;
            min_threshold = nwrong;
        }
    }

    return best_match; // Return the best match nibble (0xFF if no match found)
}

// Finite State Machine methods
void ieee802154_packet_sink_impl::enter_search_preamble()
{
    if (d_state != state::SEARCH_PREAMBLE) {
        d_shift_reg = 0;
        d_fill_buffer_count = 0;
    }
    d_nibble_count = 0;
    d_chip_count = 0;
    d_state = state::SEARCH_PREAMBLE;
}
void ieee802154_packet_sink_impl::enter_decode_length()
{
    d_chip_count = 0;
    d_payload_len = 0;
    d_reg_byte = 0;
    d_nibble_count = 0;
    d_shift_reg = 0;
    d_bytes_count = 0;
    d_state = state::DECODE_LENGTH;
}
void ieee802154_packet_sink_impl::enter_decode_payload()
{
    d_chip_count = 0;
    d_reg_byte = 0;
    d_nibble_count = 0;
    d_entering_payload = true;
    d_bytes_count = 0;
    d_state = state::DECODE_PAYLOAD;
}
void ieee802154_packet_sink_impl::enter_check_crc() { d_state = state::CHECK_CRC; }
void ieee802154_packet_sink_impl::process_search_preamble(uint8_t chip,
                                                          uint64_t sample_index)
{
    (void)sample_index;
    d_shift_reg = (d_shift_reg << 1) | chip; // Shift in new chip

    // Buffer yet to be filled
    if (d_fill_buffer_count < d_chip_sequence_len) {
        d_fill_buffer_count++;
        return;
    }
    // Look for the first 32-chip sequence representing a 0x0 nibble
    if (d_nibble_count == 0) {
        if (nibble_match(
                d_shift_reg, d_preamble_sequence.at(d_nibble_count), d_threshold)) {
            d_nibble_count++;
        }
        return;
    }
    // Start processing every 32-chip block
    if (++d_chip_count < 32) {
        return; // Still collecting chips to unpack a nibble
    }
    d_chip_count = 0; // Reset chip counter

    if (!nibble_match(d_shift_reg, d_preamble_sequence.at(d_nibble_count), d_threshold)) {
        enter_search_preamble();
        return; // The read chip sequence doesn't match the position in the preamble
    }
    // We found the entire preamble sequence
    if (++d_nibble_count >= d_preamble_sequence.size()) {
        enter_decode_length();
    }
}
void ieee802154_packet_sink_impl::process_decode_length(uint8_t chip,
                                                        uint64_t sample_index)
{
    d_shift_reg = (d_shift_reg << 1) | chip; // Shift in new chip

    // Start processing every 32-chip block
    if (++d_chip_count < 32) {
        return; // Still collecting chips to unpack a nibble
    }
    d_chip_count = 0; // Reset chip counter

    uint8_t nibble = pack_chips_to_nibble(d_shift_reg, d_threshold);

    if (nibble == 0xFF) {
        enter_search_preamble(); // No match found, reset state machine
        return;
    }

    d_reg_byte = (d_reg_byte >> 4) | (nibble << 4);

    if (++d_nibble_count < 2) {
        return; // Still collecting nibbles to unpack a byte
    }
    d_nibble_count = 0;
    if (d_bytes_count < 1) {
        d_payload_len = d_crc_included ? d_reg_byte - d_crc_len : d_reg_byte;

        if (d_payload_len > d_max_payload_len) {
            // Invalid payload length, reset state machine
            enter_search_preamble();
            return;
        }
        if (++d_bytes_count == 1) {
            enter_decode_payload();
            return;
        }
    }
}
void ieee802154_packet_sink_impl::process_decode_payload(uint8_t chip,
                                                         uint64_t sample_index)
{
    if (d_entering_payload && d_output_connected) {
        // Add a tag for the start of the payload to the current sample index
        d_sample_payload_index = sample_index;
        pmt::pmt_t tag_value = pmt::make_tuple(pmt::from_uint64(d_packet_count),
                                               pmt::from_uint64(d_payload_len));
        add_item_tag(0, d_sample_payload_index, pmt::intern("Payload start"), tag_value);
        d_entering_payload = false;
    }
}
void ieee802154_packet_sink_impl::process_check_crc(uint8_t chip, uint64_t sample_index)
{
    (void)chip;
    (void)sample_index;
}

int ieee802154_packet_sink_impl::work(int noutput_items,
                                      gr_vector_const_void_star& input_items,
                                      gr_vector_void_star& output_items)
{
    auto in = static_cast<const input_type*>(input_items[0]);
    output_type* out = nullptr;
    d_output_connected = output_items.size() > 0 && output_items[0] != nullptr;
    if (d_output_connected) {
        out = static_cast<output_type*>(output_items[0]);
    }

    for (int i = 0; i < noutput_items; i++) {
        uint8_t rx_chip = slice(in[i]);

        if (out) {
            out[i] = rx_chip; // Optional stream output: sliced data
        }

        uint64_t sample_index = nitems_read(0) + i;

        switch (d_state) {
        case state::SEARCH_PREAMBLE:
            process_search_preamble(rx_chip, sample_index);
            break;

        case state::DECODE_LENGTH:
            process_decode_length(rx_chip, sample_index);
            break;

        case state::DECODE_PAYLOAD:
            process_decode_payload(rx_chip, sample_index);
            break;

        case state::CHECK_CRC:
            process_check_crc(rx_chip, sample_index);
            break;
        }
    }
    return noutput_items; // Tell runtime system how many output items we produced.
}

} /* namespace ble */
} /* namespace gr */
