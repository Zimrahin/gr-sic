import numpy as np
import matplotlib.pyplot as plt


def triangular_wave(step: int, length: int) -> np.ndarray:
    max_val = np.iinfo(np.uint8).max
    ascend = np.arange(0, max_val + 1, step, dtype=np.uint8)
    descend = ascend[-2:0:-1]
    cycle = np.concatenate((ascend, descend))
    repeats = int(np.ceil(length / cycle.size))
    return np.tile(cycle, repeats)[:length]


class TransmitterBLE:
    def __init__(self, sample_rate, transmission_rate=1e6):
        self.sample_rate = sample_rate
        self.transmission_rate = transmission_rate
        self.sps = int(sample_rate / transmission_rate)

    def process_phy_payload(self, payload):
        return np.random.randint(0, 1, len(payload) * 8)

    def modulate(self, bits):
        num_samples = len(bits) * self.sps
        real = np.random.randn(num_samples).astype(np.float32)
        imag = np.random.randn(num_samples).astype(np.float32)
        return (real + 1j * imag).astype(np.complex64)
        # IMPORTANT: .astype(np.complex64) is necessary to avoid type issues in GNU Radio

    def modulate_from_payload(self, payload: np.ndarray) -> np.ndarray:
        """Generates IQ data from physical payload"""
        bits = self.process_phy_payload(payload)
        return self.modulate(bits)


if __name__ == "__main__":
    sample_rate = 1e6  # Example sample rate
    transmitter = TransmitterBLE(sample_rate)

    # Example payload
    payload = triangular_wave(step=2, length=255)

    # Process and modulate the payload
    bits = transmitter.process_phy_payload(payload)
    modulated_signal = transmitter.modulate(bits)

    # Triangular shape

    # Plot the modulated signal
    plt.figure(figsize=(10, 4))
    plt.plot(np.real(modulated_signal), label="In-phase")
    plt.plot(np.imag(modulated_signal), label="Quadrature")
    plt.title("Modulated Signal")
    plt.xlabel("Sample Index")
    plt.ylabel("Amplitude")
    plt.legend()
    plt.grid()
    plt.show()

    # plot the triangular waveform
    plt.figure(figsize=(10, 4))
    plt.stem(payload, label="Triangular Waveform")
    plt.title("Triangular Waveform")
    plt.xlabel("Sample Index")
    plt.ylabel("Amplitude")
    plt.legend()
    plt.grid()
    plt.show()
