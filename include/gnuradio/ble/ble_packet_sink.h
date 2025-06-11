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
 * \brief
 * \ingroup ble
 *
 */
class BLE_API ble_packet_sink : virtual public gr::sync_block
{
public:
    typedef std::shared_ptr<ble_packet_sink> sptr;

    /*!
     * \brief This block detects BLE packets in hard-decision bitstreams and publishes
     * them as PDUs. It searches for the preamble based on the base address and preamble
     * threshold. Detected packets are de-whitened and CRC checked.
     * \param base_address The base address used to generate the access code for preamble
     * detection.
     * \param preamble_threshold The maximum number of bit errors allowed in the preamble
     * detection. If the number of errors exceeds this threshold, the preamble is not
     * detected.
     * \param lfsr The initial value of the LFSR used for whitening. It should be a 7-bit
     * value (0x00 to 0x7F).
     * \param block_id An identifier for the block instance, used in the output message
     * metadata.
     */
    static sptr
    make(uint32_t base_address, uint preamble_threshold, uint8_t lfsr, uint block_id);
};

} // namespace ble
} // namespace gr

#endif /* INCLUDED_BLE_BLE_PACKET_SINK_H */
