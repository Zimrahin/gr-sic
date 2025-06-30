/* -*- c++ -*- */
/*
 * Author: Diego Badillo-San-Juan <diego.badillo-san-juan@inria.fr>
 * Copyright 2025 Inria.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include "ieee802154_packet_sink_impl.h"
#include <gnuradio/io_signature.h>

namespace gr {
namespace ble {

#pragma message("set the following appropriately and remove this warning")
using input_type = float;
ieee802154_packet_sink::sptr ieee802154_packet_sink::make(uint preamble_threshold,
                                                          uint block_id)
{
    return gnuradio::make_block_sptr<ieee802154_packet_sink_impl>(preamble_threshold,
                                                                  block_id);
}


// Constructor
ieee802154_packet_sink_impl::ieee802154_packet_sink_impl(uint preamble_threshold,
                                                         uint block_id)
    : gr::sync_block("ieee802154_packet_sink",
                     gr::io_signature::make(
                         1 /* min inputs */, 1 /* max inputs */, sizeof(input_type)),
                     gr::io_signature::make(0, 0, 0))
{
}

// Destructor
ieee802154_packet_sink_impl::~ieee802154_packet_sink_impl() {}

int ieee802154_packet_sink_impl::work(int noutput_items,
                                      gr_vector_const_void_star& input_items,
                                      gr_vector_void_star& output_items)
{
    auto in = static_cast<const input_type*>(input_items[0]);

#pragma message("Implement the signal processing in your block and remove this warning")
    // Do <+signal processing+>

    // Tell runtime system how many output items we produced.
    return noutput_items;
}

} /* namespace ble */
} /* namespace gr */
