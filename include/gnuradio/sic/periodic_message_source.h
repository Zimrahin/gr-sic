/* -*- c++ -*- */
/*
 * Author: Diego Badillo-San-Juan <diego.badillo-san-juan@inria.fr>
 * Copyright 2025 Inria.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

/*
 * References:
 * - bastibl/gr-foo (periodic_message_source_impl.cc)
 */

#ifndef INCLUDED_SIC_PERIODIC_MESSAGE_SOURCE_H
#define INCLUDED_SIC_PERIODIC_MESSAGE_SOURCE_H

#include <gnuradio/block.h>
#include <gnuradio/sic/api.h>

namespace gr {
namespace sic {

/*!
 * \brief Perioidic Message Source
 * \ingroup sic
 *
 */
class SIC_API periodic_message_source : virtual public gr::block
{
public:
    typedef std::shared_ptr<periodic_message_source> sptr;

    /*!
     * \brief This blocks sends a PMT message periodically. The optional message input
     allows to pause transmission.
     * \param message The PMT message to send periodically.
     * \param period_ms The period_ms in milliseconds between messages.
     * \param n_messages The number of messages to send before stopping. If set to -1,
     * it will send messages indefinitely.
     */
    static sptr make(pmt::pmt_t message, long period_ms, int n_messages);
};

} // namespace sic
} // namespace gr

#endif /* INCLUDED_SIC_PERIODIC_MESSAGE_SOURCE_H */
