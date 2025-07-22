# Apps

This folder contains two example applications of the Successive Interference Cancellation (SIC) block for concurrent BLE and IEEE 802.15.4 transmissions and successive demodulation.

`sic_simulation_app.grc`: Simulates transmission through an AWGN channel to test the SIC module without requiring external SDR hardware.

`sic_app.grc`: Uses two PlutoSDR devices—one as both receiver and transmitter, and the other as transmitter only—to demonstrate online SIC with two concurrent transmitters and one receiver. The SIC procedure runs in parallel with the flowgraph using Python threading.
A `slowing_factor` parameter in `sic_app.grc` downscales the sample rate and the BLE and IEEE 802.15.4 transmission rates by that factor, preventing overruns in the flowgraph and bugs caused when the computer cannot handle a high sample rate.
