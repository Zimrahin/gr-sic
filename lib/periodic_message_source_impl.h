/* -*- c++ -*- */
/*
 * Author: Diego Badillo-San-Juan <diego.badillo-san-juan@inria.fr>
 * Copyright 2025 Inria.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_SIC_PERIODIC_MESSAGE_SOURCE_IMPL_H
#define INCLUDED_SIC_PERIODIC_MESSAGE_SOURCE_IMPL_H

#include <gnuradio/sic/periodic_message_source.h>
#include <boost/asio.hpp>

namespace gr {
namespace sic {

class periodic_message_source_impl : public periodic_message_source
{
public:
    periodic_message_source_impl(pmt::pmt_t message, long period_ms, int n_messages);
    ~periodic_message_source_impl();

    void set_paused(bool paused);
    void set_period(long period_ms);
    void set_n_messages(int n_messages);

private:
    void start_timer();
    void timer_callback(const boost::system::error_code& error);
    void handle_pause(pmt::pmt_t msg);

    pmt::pmt_t d_message;
    long d_period_ms;
    std::atomic<int> d_n_messages;
    std::atomic<int> d_messages_sent;
    std::atomic<bool> d_paused;

    boost::asio::io_service d_io_service;
    boost::asio::io_service::work d_work; // Keep io_service running
    boost::asio::deadline_timer d_timer;
    boost::thread d_thread;
    std::atomic<bool>
        d_running; // Prevents scheduling new events after destruction has begun

    gr::thread::mutex d_mutex;
};

} // namespace sic
} // namespace gr

#endif /* INCLUDED_SIC_PERIODIC_MESSAGE_SOURCE_IMPL_H */
