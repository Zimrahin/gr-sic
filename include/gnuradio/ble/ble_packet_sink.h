/* -*- c++ -*- */
/*
 * Copyright 2025 Diego Badillo-San-Juan.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_BLE_BLE_PACKET_SINK_H
#define INCLUDED_BLE_BLE_PACKET_SINK_H

#include <gnuradio/ble/api.h>
#include <gnuradio/sync_block.h>

namespace gr {
namespace ble {

/*!
 * \brief <+description of block+>
 * \ingroup ble
 *
 */
class BLE_API ble_packet_sink : virtual public gr::sync_block
{
public:
    typedef std::shared_ptr<ble_packet_sink> sptr;

    /*!
     * \brief Return a shared_ptr to a new instance of ble::ble_packet_sink.
     *
     * To avoid accidental use of raw pointers, ble::ble_packet_sink's
     * constructor is in a private implementation
     * class. ble::ble_packet_sink::make is the public interface for
     * creating new instances.
     */
    static sptr
    make(uint32_t base_address, uint preamble_threshold, uint8_t lfsr, uint block_id);
};

} // namespace ble
} // namespace gr

#endif /* INCLUDED_BLE_BLE_PACKET_SINK_H */
