/* -*- c++ -*- */
/*
 * Author: Diego Badillo-San-Juan <diego.badillo-san-juan@inria.fr>
 * Copyright 2025 Inria.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_SIC_TRANSMISSION_ENABLER_H
#define INCLUDED_SIC_TRANSMISSION_ENABLER_H

#include <gnuradio/sic/api.h>
#include <gnuradio/sync_block.h>

namespace gr {
namespace sic {

/*!
 * \brief Transmission Enabler
 * \ingroup sic
 *
 */
class SIC_API transmission_enabler : virtual public gr::sync_block
{
public:
    typedef std::shared_ptr<transmission_enabler> sptr;

    /*!
     * \brief This block generates a tagged stream of zero-valued samples. It receives a
     * message input that triggers the addition of a tag in the next work call. This
     * enables the synchronisation of multiple transmissions based on the tagged stream,
     * leveraging the GNU Radio scheduler.
     * \param block_size The number of samples to output in each work call.
     */
    static sptr make(int block_size);
};

} // namespace sic
} // namespace gr

#endif /* INCLUDED_SIC_TRANSMISSION_ENABLER_H */
