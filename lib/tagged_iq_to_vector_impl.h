/* -*- c++ -*- */
/*
 * Author: Diego Badillo-San-Juan <diego.badillo-san-juan@inria.fr>
 * Copyright 2025 Inria.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_BLE_TAGGED_IQ_TO_VECTOR_IMPL_H
#define INCLUDED_BLE_TAGGED_IQ_TO_VECTOR_IMPL_H

#include <gnuradio/ble/tagged_iq_to_vector.h>

namespace gr {
namespace ble {

class tagged_iq_to_vector_impl : public tagged_iq_to_vector
{
public:
    tagged_iq_to_vector_impl(uint64_t pre_offset, uint64_t post_offset, uint64_t max_gap);
    ~tagged_iq_to_vector_impl();

    // Where all the action really happens
    int work(int noutput_items,
             gr_vector_const_void_star& input_items,
             gr_vector_void_star& output_items);

private:
    // Helper functions
    void process_tags(const std::vector<tag_t> tags); // Add packet requests based on tags

    // Variables
    uint64_t d_pre_offset;           // Sample offset before the starting tag
    uint64_t d_post_offset;          // Sample offset after the ending tag
    uint64_t d_max_gap;              // Maximum gap between tags for the same packet
    uint64_t d_buffer_size;          // Maximum size of the buffer
    std::deque<gr_complex> d_buffer; // Double-end queue to hold the samples
    uint64_t d_buffer_base; // Buffer base index, used to calculate absolute sample
                            // indices of input stream
    std::map<uint64_t, uint64_t> d_active_starts; // Store the active (not yet processed)
                                                  // start tags with their ID (counter)

    // Structure to hold pending requests for sample extraction
    // (one request per packet)
    struct request_t {
        uint64_t id;
        uint64_t window_start;
        uint64_t window_end;
    };
    std::list<request_t> d_pending_requests;
};

} // namespace ble
} // namespace gr

#endif /* INCLUDED_BLE_TAGGED_IQ_TO_VECTOR_IMPL_H */
