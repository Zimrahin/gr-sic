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
 * \brief <+description of block+>
 * \ingroup ble
 *
 */
class BLE_API tag_iq_stream : virtual public gr::block
{
public:
    typedef std::shared_ptr<tag_iq_stream> sptr;

    /*!
     * \brief Return a shared_ptr to a new instance of ble::tag_iq_stream.
     *
     * To avoid accidental use of raw pointers, ble::tag_iq_stream's
     * constructor is in a private implementation
     * class. ble::tag_iq_stream::make is the public interface for
     * creating new instances.
     */
    static sptr make(uint sps);
};

} // namespace ble
} // namespace gr

#endif /* INCLUDED_BLE_TAG_IQ_STREAM_H */
