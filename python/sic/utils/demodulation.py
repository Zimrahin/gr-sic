from gnuradio import digital, blocks, gr, analog
import numpy as np
from typing import Literal


# Computes instantaneous frequency of a complex IQ signal
def demodulate_frequency(iq_samples: np.ndarray, gain: float | int = 1) -> np.ndarray:
    """Computes instantaneous frequency of a complex IQ signal."""
    return np.diff(np.unwrap(np.angle(iq_samples))) * gain


# Hard decision
def binary_slicer(data: np.ndarray) -> np.ndarray:
    return np.where(data >= 0, 1, 0).astype(np.int8)


# Define allowed TED types
TEDType = Literal[
    "MUELLER_AND_MULLER",
    "MOD_MUELLER_AND_MULLER",
    "ZERO_CROSSING",
    "GARDNER",
    "EARLY_LATE",
    "DANDREA_AND_MENGALI_GEN_MSK",
    "MENGALI_AND_DANDREA_GMSK",
    "SIGNAL_TIMES_SLOPE_ML",
    "SIGNUM_TIMES_SLOPE_ML",
]

# Mapping from string to GNU Radio constants
TED_TYPES = {
    "MUELLER_AND_MULLER": digital.TED_MUELLER_AND_MULLER,
    "MOD_MUELLER_AND_MULLER": digital.TED_MOD_MUELLER_AND_MULLER,
    "ZERO_CROSSING": digital.TED_ZERO_CROSSING,
    "GARDNER": digital.TED_GARDNER,
    "EARLY_LATE": digital.TED_EARLY_LATE,
    "DANDREA_AND_MENGALI_GEN_MSK": digital.TED_DANDREA_AND_MENGALI_GEN_MSK,
    "MENGALI_AND_DANDREA_GMSK": digital.TED_MENGALI_AND_DANDREA_GMSK,
    "SIGNAL_TIMES_SLOPE_ML": digital.TED_SIGNAL_TIMES_SLOPE_ML,
    "SIGNUM_TIMES_SLOPE_ML": digital.TED_SIGNUM_TIMES_SLOPE_ML,
}


# Symbol synchronisation function from GNU Radio
def symbol_sync(
    input_samples: np.ndarray,
    sps: float,
    TED_gain: float = 1.0,
    loop_BW: float = 0.045,
    damping: float = 1.0,
    max_deviation: float = 1.5,
    out_sps: int = 1,
    ted_type: TEDType = "MOD_MUELLER_AND_MULLER",
) -> np.ndarray:
    """Symbol synchronisation function from GNU Radio."""
    if ted_type not in TED_TYPES:
        raise ValueError(f"Invalid TED type '{ted_type}'. Choose from {list(TED_TYPES.keys())}")

    # Convert NumPy array to GNU Radio format
    src = blocks.vector_source_f(input_samples.tolist(), False, 1, [])

    # Instantiate the symbol sync block
    symbol_sync_block = digital.symbol_sync_ff(
        TED_TYPES[ted_type],
        sps,
        loop_BW,
        damping,
        TED_gain,
        max_deviation,
        out_sps,
        digital.constellation_bpsk().base(),
        digital.IR_MMSE_8TAP,
        128,
        [],
    )

    # Sink to collect the output
    sink = blocks.vector_sink_f(1, 1024 * 4)

    # Connect blocks and run
    tb = gr.top_block()
    tb.connect(src, symbol_sync_block, sink)
    tb.run()

    return np.array(sink.data())
