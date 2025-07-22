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

#include "periodic_message_source_impl.h"
#include <gnuradio/io_signature.h>
#include <boost/bind/bind.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

namespace gr {
namespace sic {

periodic_message_source::sptr
periodic_message_source::make(pmt::pmt_t message, long period_ms, int n_messages)
{
    return gnuradio::make_block_sptr<periodic_message_source_impl>(
        message, period_ms, n_messages);
}


// Constructor
periodic_message_source_impl::periodic_message_source_impl(pmt::pmt_t message,
                                                           long period_ms,
                                                           int n_messages)
    : gr::block("periodic_message_source",
                gr::io_signature::make(0, 0, 0),
                gr::io_signature::make(0, 0, 0)),
      d_message(message),
      d_period_ms(period_ms),
      d_n_messages(n_messages),
      d_messages_sent(0),
      d_paused(false),
      d_work(d_io_service),
      d_timer(d_io_service),
      d_running(true)
{
    // Message ports
    message_port_register_out(pmt::mp("out"));
    message_port_register_in(pmt::mp("pause"));
    set_msg_handler(pmt::mp("pause"),
                    [this](pmt::pmt_t msg) { this->handle_pause(msg); });

    // Start timer thread
    d_thread = boost::thread([this]() {
        start_timer();
        d_io_service.run();
    });
}

// Destructor
periodic_message_source_impl::~periodic_message_source_impl()
{
    d_running = false; // Prevents scheduling new events
    d_io_service.stop();
    d_timer.cancel();
    if (d_thread.joinable()) {
        d_thread.join();
    }
}

// Schedules the next timer expiration based on current period
void periodic_message_source_impl::start_timer()
{
    if (d_running && !d_paused) {
        d_timer.expires_from_now(boost::posix_time::milliseconds(d_period_ms));
        d_timer.async_wait(boost::bind(&periodic_message_source_impl::timer_callback,
                                       this,
                                       boost::placeholders::_1));
    }
}

// Send message when timer expires
void periodic_message_source_impl::timer_callback(const boost::system::error_code& error)
{
    if (!error && d_running) {
        // Send message if not paused and within message limit
        if (!d_paused && (d_n_messages == -1 || d_messages_sent < d_n_messages)) {
            message_port_pub(pmt::mp("out"), d_message);
            d_messages_sent++;
        }

        // Restart timer if block is still running
        if (d_running && (d_n_messages == -1 || d_messages_sent < d_n_messages)) {
            start_timer();
        }
    }
}

// Input message handler for pause control
void periodic_message_source_impl::handle_pause(pmt::pmt_t msg)
{
    if (pmt::is_pair(msg)) {
        pmt::pmt_t value = pmt::cdr(msg);
        if (pmt::is_integer(value)) {
            long state = pmt::to_long(value);
            set_paused(state != 0);
        } else if (pmt::is_bool(value)) {
            bool state = pmt::to_bool(value);
            set_paused(state);
        }
    }
}

// Setters
void periodic_message_source_impl::set_paused(bool paused)
{
    gr::thread::scoped_lock guard(d_mutex);
    d_paused = paused;

    // Restart timer if unpaused
    if (!paused && d_running) {
        d_io_service.post([this]() {
            d_timer.cancel();
            start_timer();
        });
    }
}
void periodic_message_source_impl::set_period(long period_ms)
{
    gr::thread::scoped_lock guard(d_mutex);
    d_period_ms = period_ms;

    // Restart with new period if running
    if (d_running && !d_paused) {
        d_io_service.post([this]() {
            d_timer.cancel();
            start_timer();
        });
    }
}
void periodic_message_source_impl::set_n_messages(int n_messages)
{
    gr::thread::scoped_lock guard(d_mutex);
    d_n_messages = n_messages;
}


} /* namespace sic */
} /* namespace gr */
