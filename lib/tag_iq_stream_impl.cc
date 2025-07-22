/* -*- c++ -*- */
/*
 * Author: Diego Badillo-San-Juan <diego.badillo-san-juan@inria.fr>
 * Copyright 2025 Inria.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include "tag_iq_stream_impl.h"
#include <gnuradio/io_signature.h>

namespace gr {
namespace sic {

using input_type_0 = gr_complex;
using input_type_1 = uint8_t;

using output_type = gr_complex;

tag_iq_stream::sptr tag_iq_stream::make(uint sps)
{
    return gnuradio::make_block_sptr<tag_iq_stream_impl>(sps);
}

// Constructor
tag_iq_stream_impl::tag_iq_stream_impl(uint sps)
    : gr::block("tag_iq_stream",
                gr::io_signature::makev(
                    2, 2, std::vector<int>{ sizeof(input_type_0), sizeof(input_type_1) }),
                gr::io_signature::make(1, 1, sizeof(output_type))),
      d_sps(sps)
{
    set_output_multiple(d_sps);
    set_tag_propagation_policy(TPP_CUSTOM);
}

// Destructor
tag_iq_stream_impl::~tag_iq_stream_impl() {}

void tag_iq_stream_impl::forecast(int noutput_items, gr_vector_int& ninput_items_required)
{
    // Set the relationship between input and output items
    ninput_items_required[0] = noutput_items;
    ninput_items_required[1] = noutput_items / d_sps;
}

void tag_iq_stream_impl::set_sps(uint sps)
{
    gr::thread::scoped_lock guard(d_mutex);
    d_sps = sps;
    set_output_multiple(d_sps);
    set_relative_rate(1, d_sps);
}

int tag_iq_stream_impl::general_work(int noutput_items,
                                     gr_vector_int& ninput_items,
                                     gr_vector_const_void_star& input_items,
                                     gr_vector_void_star& output_items)
{
    auto in_iq = static_cast<const input_type_0*>(input_items[0]);
    auto out_iq = static_cast<output_type*>(output_items[0]);


    gr::thread::scoped_lock lock(d_mutex);
    const uint current_sps = d_sps;


    // Ensure ratio between IQ input and symbol input of sps to 1
    int n_symbols = std::min(static_cast<int>(ninput_items[1]),
                             std::min(static_cast<int>(ninput_items[0] / current_sps),
                                      static_cast<int>(noutput_items / current_sps)));
    if (n_symbols == 0) {
        consume(0, 0);
        consume(1, 0);
        return 0; // Not enough IQ data for a full symbol
    }

    // Copy IQ samples
    int n_produced = n_symbols * current_sps;
    memcpy(out_iq, in_iq, n_produced * sizeof(output_type));

    // Get symbol stream tags
    std::vector<tag_t> symbol_tags;
    uint sym_port = 1;
    get_tags_in_range(
        symbol_tags, sym_port, nitems_read(sym_port), nitems_read(sym_port) + n_symbols);

    // Copy tags to IQ steam
    for (auto& tag : symbol_tags) {
        uint64_t iq_tag_offset =
            (tag.offset - nitems_read(1)) * current_sps + nitems_read(0);
        add_item_tag(0, iq_tag_offset, tag.key, tag.value);
    }

    // Consume inputs
    consume(0, n_produced);
    consume(1, n_symbols);

    return noutput_items; // Tell runtime system how many output items we produced.
}

} /* namespace sic */
} /* namespace gr */
