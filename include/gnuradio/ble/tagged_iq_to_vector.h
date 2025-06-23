/* -*- c++ -*- */
/*
 * Author: Diego Badillo-San-Juan <diego.badillo-san-juan@inria.fr>
 * Copyright 2025 Inria.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_BLE_TAGGED_IQ_TO_VECTOR_H
#define INCLUDED_BLE_TAGGED_IQ_TO_VECTOR_H

#include <gnuradio/ble/api.h>
#include <gnuradio/sync_block.h>

namespace gr {
namespace ble {

/*!
 * \brief <+description of block+>
 * \ingroup ble
 *
 */
class BLE_API tagged_iq_to_vector : virtual public gr::sync_block
{
public:
    typedef std::shared_ptr<tagged_iq_to_vector> sptr;

    /*!
     * \brief Return a shared_ptr to a new instance of ble::tagged_iq_to_vector.
     *
     * To avoid accidental use of raw pointers, ble::tagged_iq_to_vector's
     * constructor is in a private implementation
     * class. ble::tagged_iq_to_vector::make is the public interface for
     * creating new instances.
     */
    static sptr make(uint64_t pre_offset, uint64_t post_offset, uint64_t max_gap);
};

} // namespace ble
} // namespace gr

#endif /* INCLUDED_BLE_TAGGED_IQ_TO_VECTOR_H */
