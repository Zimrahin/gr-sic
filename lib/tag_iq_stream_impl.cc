
#include "tag_iq_stream_impl.h"
#include <gnuradio/io_signature.h>

namespace gr {
namespace ble {

using input_type_0 = gr_complex;
using input_type_1 = uint8_t;

using output_type = gr_complex;

tag_iq_stream::sptr tag_iq_stream::make(uint sps)
{
    return gnuradio::make_block_sptr<tag_iq_stream_impl>(sps);
}


// Constructor
tag_iq_stream_impl::tag_iq_stream_impl(uint sps)
    : gr::block("tag_iq_stream",
                gr::io_signature::makev(
                    2, 2, std::vector<int>{ sizeof(input_type_0), sizeof(input_type_1) }),
                gr::io_signature::make(1, 1, sizeof(output_type))),
      d_sps(sps)
{
    // set_relative_rate(d_sps, 1);
    set_output_multiple(d_sps);
}

// Destructor
tag_iq_stream_impl::~tag_iq_stream_impl() {}

void tag_iq_stream_impl::forecast(int noutput_items, gr_vector_int& ninput_items_required)
{
    ninput_items_required[0] = noutput_items;
    ninput_items_required[1] = (noutput_items + d_sps - 1) / d_sps; // Ceiling division
}

int tag_iq_stream_impl::general_work(int noutput_items,
                                     gr_vector_int& ninput_items,
                                     gr_vector_const_void_star& input_items,
                                     gr_vector_void_star& output_items)
{
    auto in_iq = static_cast<const input_type_0*>(input_items[0]);
    // auto in_sym = static_cast<const input_type_1*>(input_items[1]);
    auto out_iq = static_cast<output_type*>(output_items[0]);

    // Forward IQ samples
    // for (int i = 0; i < noutput_items; i++) {
    //     out_iq[i] = in_iq[i];
    // }

    // Get symbol stream tags
    std::vector<tag_t> tags;
    uint sym_port = 1;
    get_tags_in_range(tags,
                      sym_port,
                      nitems_read(sym_port),
                      nitems_read(sym_port) + ninput_items[sym_port]);


    // Copy tags to IQ steam
    for (auto& t : tags) {
        std::cout << "Tag found: " << pmt::write_string(t.key)
                  << " at offset: " << t.offset << std::endl;
        // uint64_t sym_idx = t.offset; // Absolute tag index
        // uint64_t iq_idx = sym_idx * d_sps;
        // int64_t relative_idx = int64_t(iq_idx) - int64_t(nitems_written(0));
        // if (0 <= relative_idx && relative_idx < noutput_items) {
        // add_item_tag(0,
        //  iq_idx,
        //  t.key,  // same tag key
        //  t.value // same tag value
        // );
        // }
    }

    consume(0, ninput_items[0]);
    consume(1, ninput_items[1]);

    return noutput_items; // Tell runtime system how many output items we produced.
}

} /* namespace ble */
} /* namespace gr */
