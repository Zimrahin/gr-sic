/* -*- c++ -*- */
/*
 * Author: Diego Badillo-San-Juan <diego.badillo-san-juan@inria.fr>
 * Copyright 2025 Inria.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include "tag_stream_from_message_impl.h"
#include <gnuradio/io_signature.h>

namespace gr {
namespace ble {

using input_type = gr_complex;
using output_type = gr_complex;

tag_stream_from_message::sptr tag_stream_from_message::make()
{
    return gnuradio::make_block_sptr<tag_stream_from_message_impl>();
}


// Constructor
tag_stream_from_message_impl::tag_stream_from_message_impl()
    : gr::sync_block("tag_stream_from_message",
                     gr::io_signature::make(1, 1, sizeof(input_type)),
                     gr::io_signature::make(1, 1, sizeof(output_type)))
{
    // PMT message ports
    message_port_register_in(pmt::mp("in"));
    message_port_register_out(pmt::mp("out"));
    set_msg_handler(pmt::mp("in"), [this](pmt::pmt_t msg) { handle_message(msg); });
}

// Destructor
tag_stream_from_message_impl::~tag_stream_from_message_impl() {}

void tag_stream_from_message_impl::handle_message(pmt::pmt_t msg)
{
    // We expect msg = cons(meta_dict, payload_blob) from packet sink
    if (!pmt::is_pair(msg)) {
        return;
    }
    pmt::pmt_t meta = pmt::car(msg);
    pmt::pmt_t sample_index =
        pmt::dict_ref(meta, pmt::intern("Payload start sample"), pmt::PMT_NIL);
    if (!pmt::is_uint64(sample_index)) {
        return;
    }
    // uint64_t sample_index = pmt::to_uint64(sample_index);

    // Tag dictionary
    pmt::pmt_t tag_dict = pmt::make_dict();
    tag_dict = pmt::dict_add(tag_dict, pmt::intern("sample_index"), sample_index);
    tag_dict = pmt::dict_add(
        tag_dict, pmt::intern("written_index"), pmt::from_uint64(nitems_written(0)));

    // Tag the next IQ sample
    add_item_tag(0, nitems_written(0), pmt::intern("test_key"), tag_dict);

    // Forward the message
    message_port_pub(pmt::mp("out"), msg);
}

int tag_stream_from_message_impl::work(int noutput_items,
                                       gr_vector_const_void_star& input_items,
                                       gr_vector_void_star& output_items)
{
    auto in = static_cast<const input_type*>(input_items[0]);
    auto out = static_cast<output_type*>(output_items[0]);

    // Forward input to output
    for (int i = 0; i < noutput_items; i++) {
        out[i] = in[i];
    }


    return noutput_items; // Tell runtime system how many output items we produced.
}

} /* namespace ble */
} /* namespace gr */
