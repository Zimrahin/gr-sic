/* -*- c++ -*- */
/*
 * Author: Diego Badillo-San-Juan <diego.badillo-san-juan@inria.fr>
 * Copyright 2025 Inria.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_BLE_IEEE802154_PACKET_SINK_IMPL_H
#define INCLUDED_BLE_IEEE802154_PACKET_SINK_IMPL_H

#include <gnuradio/ble/ieee802154_packet_sink.h>

namespace gr {
namespace ble {

class ieee802154_packet_sink_impl : public ieee802154_packet_sink
{
private:
    // Nothing to declare in this block.

public:
    ieee802154_packet_sink_impl(uint preamble_threshold, uint block_id);
    ~ieee802154_packet_sink_impl();

    // Where all the action really happens
    int work(int noutput_items,
             gr_vector_const_void_star& input_items,
             gr_vector_void_star& output_items);
};

} // namespace ble
} // namespace gr

#endif /* INCLUDED_BLE_IEEE802154_PACKET_SINK_IMPL_H */
