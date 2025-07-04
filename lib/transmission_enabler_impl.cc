/* -*- c++ -*- */
/*
 * Author: Diego Badillo-San-Juan <diego.badillo-san-juan@inria.fr>
 * Copyright 2025 Inria.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include "transmission_enabler_impl.h"
#include <gnuradio/io_signature.h>

namespace gr {
namespace sic {

using output_type = gr_complex;

transmission_enabler::sptr transmission_enabler::make(int block_size)
{
    return gnuradio::make_block_sptr<transmission_enabler_impl>(block_size);
}

// Constructor
transmission_enabler_impl::transmission_enabler_impl(int block_size)
    : gr::sync_block("transmission_enabler",
                     gr::io_signature::make(0, 0, 0),
                     gr::io_signature::make(1, 1, sizeof(output_type))),
      d_block_size(block_size),
      d_trigger_pending(false)
{
    // Enforce fixed noutput_items
    set_output_multiple(block_size);
    set_min_noutput_items(block_size);
    set_max_noutput_items(block_size);

    // Set input message port
    message_port_register_in(pmt::mp("trigger"));
    set_msg_handler(pmt::mp("trigger"),
                    [this](pmt::pmt_t msg) { this->handle_trigger(msg); });
}

// Destructor
transmission_enabler_impl::~transmission_enabler_impl() {}

void transmission_enabler_impl::handle_trigger(pmt::pmt_t msg)
{
    d_trigger_pending = true;
}

int transmission_enabler_impl::work(int noutput_items,
                                    gr_vector_const_void_star& input_items,
                                    gr_vector_void_star& output_items)
{
    auto out = static_cast<output_type*>(output_items[0]);

    std::memset(out, 0, noutput_items * sizeof(gr_complex)); // All-zero output

    // Add tag if trigger is pending
    if (d_trigger_pending.exchange(false)) {
        add_item_tag(0, nitems_written(0), pmt::mp("trigger"), pmt::PMT_T);
    }

    return noutput_items;
}

} /* namespace sic */
} /* namespace gr */
