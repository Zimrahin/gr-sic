/* -*- c++ -*- */
/*
 * Author: Diego Badillo-San-Juan <diego.badillo-san-juan@inria.fr>
 * Copyright 2025 Inria.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_SIC_TRANSMISSION_ENABLER_IMPL_H
#define INCLUDED_SIC_TRANSMISSION_ENABLER_IMPL_H

#include <gnuradio/sic/transmission_enabler.h>

namespace gr {
namespace sic {

class transmission_enabler_impl : public transmission_enabler
{
public:
    transmission_enabler_impl(int block_size);
    ~transmission_enabler_impl();

    int work(int noutput_items,
             gr_vector_const_void_star& input_items,
             gr_vector_void_star& output_items);

private:
    void handle_trigger(pmt::pmt_t msg);

    const int d_block_size;
    std::atomic<bool> d_trigger_pending;
};

} // namespace sic
} // namespace gr

#endif /* INCLUDED_SIC_TRANSMISSION_ENABLER_IMPL_H */
