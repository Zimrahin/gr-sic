/* -*- c++ -*- */
/*
 * Author: Diego Badillo-San-Juan <diego.badillo-san-juan@inria.fr>
 * Copyright 2025 Inria.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_SIC_TAGGED_IQ_TO_VECTOR_H
#define INCLUDED_SIC_TAGGED_IQ_TO_VECTOR_H

#include <gnuradio/sic/api.h>
#include <gnuradio/sync_block.h>

namespace gr {
namespace sic {

/*!
 * \brief Tagged IQ to Vector
 * \ingroup sic
 *
 */
class SIC_API tagged_iq_to_vector : virtual public gr::sync_block
{
public:
    typedef std::shared_ptr<tagged_iq_to_vector> sptr;

    /*!
     * \brief This blocks receives a stream of IQ samples and extracts packets based on
     * tags. It buffers the IQ samples and allows for extraction of packets based on start
     * and end tags. The block outputs a PMT message containing the extracted samples as a
     * vector for further processing.
     * \param pre_offset The number of samples before the start tag to include in the
     * output.
     * \param post_offset The number of samples after the end tag to include in the
     * output.
     * \param max_gap The maximum gap between start and end tags to consider a valid
     * packet
     */
    static sptr make(uint64_t pre_offset, uint64_t post_offset, uint64_t max_gap);
};

} // namespace sic
} // namespace gr

#endif /* INCLUDED_SIC_TAGGED_IQ_TO_VECTOR_H */
