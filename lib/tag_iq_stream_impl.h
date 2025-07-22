/* -*- c++ -*- */
/*
 * Author: Diego Badillo-San-Juan <diego.badillo-san-juan@inria.fr>
 * Copyright 2025 Inria.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */


#ifndef INCLUDED_SIC_TAG_IQ_STREAM_IMPL_H
#define INCLUDED_SIC_TAG_IQ_STREAM_IMPL_H

#include <gnuradio/sic/tag_iq_stream.h>

namespace gr {
namespace sic {

class tag_iq_stream_impl : public tag_iq_stream
{
public:
    tag_iq_stream_impl(uint sps);
    ~tag_iq_stream_impl();

    void forecast(int noutput_items, gr_vector_int& ninput_items_required);

    int general_work(int noutput_items,
                     gr_vector_int& ninput_items,
                     gr_vector_const_void_star& input_items,
                     gr_vector_void_star& output_items);

    uint sps() const override { return d_sps; }
    void set_sps(uint sps) override;

private:
    uint d_sps; // Samples per symbol
    gr::thread::mutex d_mutex;
};

} // namespace sic
} // namespace gr

#endif /* INCLUDED_SIC_TAG_IQ_STREAM_IMPL_H */
