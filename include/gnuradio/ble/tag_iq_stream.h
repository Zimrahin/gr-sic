/* -*- c++ -*- */
/*
 * Author: Diego Badillo-San-Juan <diego.badillo-san-juan@inria.fr>
 * Copyright 2025 Inria.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_BLE_TAG_IQ_STREAM_H
#define INCLUDED_BLE_TAG_IQ_STREAM_H

#include <gnuradio/ble/api.h>
#include <gnuradio/block.h>

namespace gr {
namespace ble {

/*!
 * \brief
 * \ingroup ble
 *
 */
class BLE_API tag_iq_stream : virtual public gr::block
{
public:
    typedef std::shared_ptr<tag_iq_stream> sptr;

    /*!
     * \brief This block tags an IQ stream with a symbol stream coming from the
     * demodulation of the input IQ stream. Tagging packets in the IQ stream allows for
     * post-processing of the IQ data, such as Successive Interference Cancellation (SIC).
     * \param sps Samples per symbol. This parameter defines the ratio of data rates
     * between IQ imput and symbol input. The block expects the IQ input to be at a higher
     * rate than the symbol input by a factor of `sps`.
     */
    static sptr make(uint sps);
};

} // namespace ble
} // namespace gr

#endif /* INCLUDED_BLE_TAG_IQ_STREAM_H */
