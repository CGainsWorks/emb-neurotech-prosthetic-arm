# Documentation

| | |
|---|---|
| [architecture.md](architecture.md) | How the pieces fit together and three design decisions worth arguing about |
| [dh_parameters.md](dh_parameters.md) | The kinematic derivation: the DH table, the closed-form inverse solution, the singularities and the inertial parameters |
| [CONTROL_LOOP_TIMING.md](CONTROL_LOOP_TIMING.md) | Contraction to movement, with every number labelled measured, simulated or estimated |
| [INTEGRATION.md](INTEGRATION.md) | The contract with `ml-semg-gesture-classifier` and the three places it does not currently hold |
| [SAFETY.md](SAFETY.md) | The hazards the design accounts for and the mechanism addressing each. A design record, not a risk management file |
| [CLINICAL_LIMITATIONS.md](CLINICAL_LIMITATIONS.md) | Why this is a research prototype and not a prosthesis. Read this before drawing any conclusion from the repository |

Elsewhere:

| | |
|---|---|
| [`../kinematics/README.md`](../kinematics/README.md) | The maths library and how it is tested |
| [`../firmware/README.md`](../firmware/README.md) | The firmware, its split and the two numbers it estimates |
| [`../firmware/docs/PINOUT.md`](../firmware/docs/PINOUT.md) | Pin assignment, including the conflict that moved the servo outputs to port E |
| [`../firmware/docs/BRING_UP.md`](../firmware/docs/BRING_UP.md) | Twenty steps, none of them performed |
| [`../hardware/semg-servo-board/docs/signal_conditioning.md`](../hardware/semg-servo-board/docs/signal_conditioning.md) | The three analogue stages and why each is there |
| [`../hardware/semg-servo-board/docs/noise_analysis.md`](../hardware/semg-servo-board/docs/noise_analysis.md) | The noise budget and the conclusion the design turns on |
| [`../hardware/semg-servo-board/docs/thermal_budget.md`](../hardware/semg-servo-board/docs/thermal_budget.md) | Where the watts go and the three protections at three speeds |

Every number in any of these comes from a script in the repository.
`make check` verifies that they still agree with what those scripts produce
and CI runs it on every push. None of them was measured on hardware, because
there is none; [`../STATUS.md`](../STATUS.md) is the authority on what exists.
