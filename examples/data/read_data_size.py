import numpy as np
import click

@click.command()
@click.argument('file_path')
def read_data_size(file_path):
    """Read a .dat file of complex64 values and print the number of samples."""
    try:
        data = np.fromfile(file_path, dtype=np.complex64)
        print(f"Length: {len(data)} samples")
    except FileNotFoundError:
        print(f"File not found: {file_path}")
    except Exception as e:
        print(f"Error reading file: {e}")

if __name__ == '__main__':
    read_data_size()

