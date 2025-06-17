/* -*- c++ -*- */
/*
 * Author: Diego Badillo-San-Juan <diego.badillo-san-juan@inria.fr>
 * Copyright 2025 Inria.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_BLE_TAG_STREAM_FROM_MESSAGE_H
#define INCLUDED_BLE_TAG_STREAM_FROM_MESSAGE_H

#include <gnuradio/ble/api.h>
#include <gnuradio/sync_block.h>

namespace gr {
namespace ble {

/*!
 * \brief
 * \ingroup ble
 *
 */
class BLE_API tag_stream_from_message : virtual public gr::sync_block
{
public:
    typedef std::shared_ptr<tag_stream_from_message> sptr;

    /*!
     * \brief This block adds a tag to a stream when a input message is received.
     */
    static sptr make();
};

} // namespace ble
} // namespace gr

#endif /* INCLUDED_BLE_TAG_STREAM_FROM_MESSAGE_H */
