/* -*- c++ -*- */
/*
 * Author: Diego Badillo-San-Juan <diego.badillo-san-juan@inria.fr>
 * Copyright 2025 Inria.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include "tagged_iq_to_vector_impl.h"
#include <gnuradio/io_signature.h>

namespace gr {
namespace ble {

using input_type = gr_complex;

tagged_iq_to_vector::sptr
tagged_iq_to_vector::make(uint64_t pre_offset, uint64_t post_offset, uint64_t max_gap)
{
    return gnuradio::make_block_sptr<tagged_iq_to_vector_impl>(
        pre_offset, post_offset, max_gap);
}

// Constructor
tagged_iq_to_vector_impl::tagged_iq_to_vector_impl(uint64_t pre_offset,
                                                   uint64_t post_offset,
                                                   uint64_t max_gap)
    : gr::sync_block("tagged_iq_to_vector",
                     gr::io_signature::make(1, 1, sizeof(input_type)),
                     gr::io_signature::make(0, 0, 0)),
      d_pre_offset(pre_offset),
      d_post_offset(post_offset),
      d_max_gap(max_gap)
{
    // Initialise buffer size based on input parameters
    d_buffer_size = d_pre_offset + d_max_gap + d_pre_offset;
    d_base = 0;

    // PMT message output port
    message_port_register_out(pmt::mp("out"));
    set_tag_propagation_policy(
        TPP_DONT); // Scheduler doesn't propagate tags from in- to output
}

// Destructor
tagged_iq_to_vector_impl::~tagged_iq_to_vector_impl() {}

// Add packet requests based on tags
void tagged_iq_to_vector_impl::process_tags(const std::vector<tag_t> tags)
{
    for (const auto& tag : tags) {
        if (pmt::eq(tag.key, pmt::intern("Payload start")) && pmt::is_tuple(tag.value)) {
            uint64_t id = pmt::to_uint64(pmt::tuple_ref(tag.value, 0));

            // Look for the found ID in the active starts
            if (d_active_starts.find(id) == d_active_starts.end()) {
                // New start tag, store it
                d_active_starts[id] = tag.offset;
            }
        } else if (pmt::eq(tag.key, pmt::intern("CRC check")) &&
                   pmt::is_tuple(tag.value)) {
            uint64_t id = pmt::to_uint64(pmt::tuple_ref(tag.value, 0));

            auto iterator = d_active_starts.find(id);
            if (iterator == d_active_starts.end()) {
                // No start tag found for this ID, ignore it
                continue;
            }
            uint64_t start_tag_offset = iterator->second;
            uint64_t gap = tag.offset - start_tag_offset;
            if (gap > d_max_gap) {
                // Gap too large, ignore this paquet
                d_active_starts.erase(iterator);
                continue;
            }
            // Define the window limits to extract samples (gap + pre and post offsets)
            uint64_t window_start =
                (start_tag_offset > d_pre_offset) ? start_tag_offset - d_pre_offset : 0;
            uint64_t window_end = tag.offset + d_post_offset;

            // Append the window extraction request to pending requests
            d_pending_requests.push_back({ id, window_start, window_end });
            std::cout << "Requesting samples for ID: " << id << ", Window: ["
                      << window_start << ", " << window_end << "]\n";

            // Tag processed, remove it from active starts
            d_active_starts.erase(iterator);
        }
    }
}

int tagged_iq_to_vector_impl::work(int noutput_items,
                                   gr_vector_const_void_star& input_items,
                                   gr_vector_void_star& output_items)
{
    auto in = static_cast<const input_type*>(input_items[0]);
    int ninput = noutput_items;
    uint64_t start_abs = nitems_read(0);
    uint64_t end_abs = start_abs + ninput;

    // Buffer incoming samples
    for (int i = 0; i < ninput; i++) {
        d_buffer.push_back(in[i]);
        if (d_buffer.size() > d_buffer_size) {
            d_buffer.pop_front();
            d_base++;
        }
    }

    // Process incoming tags
    std::vector<tag_t> tags;
    get_tags_in_range(tags, 0, start_abs, end_abs);
    process_tags(tags);

    return noutput_items; // Tell runtime system how many output items we produced
}

} /* namespace ble */
} /* namespace gr */
