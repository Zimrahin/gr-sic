/* -*- c++ -*- */
/*
 * Author: Diego Badillo-San-Juan <diego.badillo-san-juan@inria.fr>
 * Copyright 2025 Inria.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_SIC_tag_aligner_H
#define INCLUDED_SIC_tag_aligner_H

#include <gnuradio/block.h>
#include <gnuradio/sic/api.h>

namespace gr {
namespace sic {

/*!
 * \brief Tag IQ Stream
 * \ingroup sic
 *
 */
class SIC_API tag_aligner : virtual public gr::block
{
public:
    typedef std::shared_ptr<tag_aligner> sptr;

    /*!
     * \brief This block copies tags from a low-rate input stream to a high-rate input
     * stream (`sps` times higher). For example, tagging packets in the IQ stream allows
     * for post-processing of the IQ data, such as Successive Interference Cancellation
     * (SIC).
     * \param sps Samples per symbol. This parameter defines the ratio of data rates
     * between IQ imput and symbol input. The block expects the IQ input to be at a higher
     * rate than the symbol input by a factor of `sps`.
     * \param type_size_high Size of high-rate input stream, e.g. 8 for gr_complex
     */
    static sptr make(uint sps, size_t type_size_high);

    virtual uint sps() const = 0;
    virtual void set_sps(uint sps) = 0;
};

} // namespace sic
} // namespace gr

#endif /* INCLUDED_SIC_tag_aligner_H */
