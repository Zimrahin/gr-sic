/* -*- c++ -*- */
/*
 * Author: Diego Badillo-San-Juan <diego.badillo-san-juan@inria.fr>
 * Copyright 2025 Inria.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_SIC_IEEE802154_PACKET_SINK_H
#define INCLUDED_SIC_IEEE802154_PACKET_SINK_H

#include <gnuradio/sic/api.h>
#include <gnuradio/sync_block.h>

namespace gr {
namespace sic {

/*!
 * \brief IEEE 802.15.4 Packet Sink
 * \ingroup sic
 *
 */
class SIC_API ieee802154_packet_sink : virtual public gr::sync_block
{
public:
    typedef std::shared_ptr<ieee802154_packet_sink> sptr;

    /*!
     * \brief This block detects IEEE 802.15.4 packets in hard-decision bitstreams and
     * publishes them as PDUs. It searches for the preamble based on the preamble
     * threshold. Detected packets are CRC checked (last two bytes of the payload).
     * \param preamble_threshold The maximum number of chip errors allowed in the preamble
     * detection. If the number of errors exceeds this threshold, the preamble is not
     * detected.
     * \param crc_included If true, the CRC is included in the payload.
     * \param block_id An identifier for the block instance, used in the output message
     * metadata.
     */
    static sptr make(uint preamble_threshold, bool crc_included, uint block_id);
};

} // namespace sic
} // namespace gr

#endif /* INCLUDED_SIC_IEEE802154_PACKET_SINK_H */
