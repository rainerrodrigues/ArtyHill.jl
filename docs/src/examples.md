# Example: cycloidal drive, from geometry to controller

[`examples/cycloidal_drive.jl`](https://github.com/rainerrodrigues/ArtyHill.jl/blob/main/examples/cycloidal_drive.jl)
walks through the whole prototyping loop for a robot joint with a cycloidal reducer.

1. **Geometry → model.** The reduction ratio and the output backlash come from the
   reducer design: `CycloidCore.backlash(DriveParams())` from
   [cycloidal-generator](https://github.com/rainerrodrigues/cycloidal-generator)
   when it is installed (9:1, 82.5 arcmin for the default Ø80 mm design), otherwise
   from the same precomputed numbers.
2. **Simulation.** Motor → backlash + compliance → load with viscous and Coulomb
   friction, as a [`NonlinearPlant`](@ref), run through [`run_realtime_loop!`](@ref)
   with an [`SPSCRing`](@ref) recording telemetry.
3. **Identification from telemetry.**
   - *Backlash* from the hysteresis between the motor and load encoders under a slow
     triangle motion: 84.8 arcmin identified vs 82.5 from the geometry. The
     difference is the elastic wind-up of the transmission.
   - *Inertias, damping and friction* by least squares on the energy balance of the
     two masses, `N·u = Jm·N²·φ̈m + bm·N²·φ̇m + JL·θ̈l + bL·θ̇l + Fc·sgn(θ̇l)`. Adding
     the motor and load equations cancels the gear torque, so this holds inside and
     outside the backlash. All five parameters come back within 2 %.
4. **Controller sweep.** For ωn = 25, 40 and 60 rad/s, PD gains are computed from the
   identified model, and two feedback structures follow the same 1 s minimum-jerk
   move:

   | ωn [rad/s] | load-side PD (RMS / p-p, mrad) | motor-side PD + backlash compensation |
   | --- | --- | --- |
   | 25 | 1.64 / 1.73 | 1.35 / 1.75 |
   | 40 | 0.09 / 0.19 | 0.27 / 0.20 |
   | 60 | 3.28 / **10.45** (hunting) | 0.34 / 0.01 |

   Load-side feedback puts the backlash inside the loop. It can be excellent, as at
   ωn = 40, but it hunts at higher bandwidth. Motor-side feedback keeps the backlash
   outside the loop and stays quiet across the sweep, at the cost of a small static
   offset that only the load encoder could remove. The example reports these numbers
   rather than declaring a winner: the right structure depends on your bandwidth
   and accuracy targets.

Run it with:

```bash
julia --threads=2 --project=examples examples/cycloidal_drive.jl
```

The test suite runs the same script and checks the identification accuracy.
