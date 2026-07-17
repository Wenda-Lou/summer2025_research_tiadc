# test_calibration.py

import numpy as np

from calibration import calculate_calibration_error

N = 2032

reference = np.sin(
    np.linspace(
        0,
        20*np.pi,
        N
    )
)

adc = 1.2*reference + 25

result = calculate_calibration_error(
    adc,
    reference,
)

print(result)