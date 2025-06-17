/* -*- c++ -*- */
/*
 * Author: Diego Badillo-San-Juan <diego.badillo-san-juan@inria.fr>
 * Copyright 2025 Inria.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_BLE_TAG_STREAM_FROM_MESSAGE_IMPL_H
#define INCLUDED_BLE_TAG_STREAM_FROM_MESSAGE_IMPL_H

#include <gnuradio/ble/tag_stream_from_message.h>

namespace gr {
namespace ble {

class tag_stream_from_message_impl : public tag_stream_from_message
{
public:
    tag_stream_from_message_impl();
    ~tag_stream_from_message_impl();

    // Called for each chunk of data in the input stream
    int work(int noutput_items,
             gr_vector_const_void_star& input_items,
             gr_vector_void_star& output_items);

private:
    void handle_message(pmt::pmt_t msg);
};

} // namespace ble
} // namespace gr

#endif /* INCLUDED_BLE_TAG_STREAM_FROM_MESSAGE_IMPL_H */
