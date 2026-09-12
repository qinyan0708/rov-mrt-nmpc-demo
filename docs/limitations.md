# Limitations and research extensions

## Current evidence boundary

The repository supports a reproducible engineering demonstration for one
carefully designed opposite-side two-obstacle scenario. It does not establish
general collision avoidance, recursive feasibility, robustness to perception
error, or safety of a physical mining vehicle.

## Highest-value extensions

1. **Online topology decision:** replace the prescribed corridor with an
   online left/right passing-side or homotopy-class decision layer.
2. **Uncertain prediction:** estimate obstacle state and propagate bounded,
   stochastic, or multimodal prediction uncertainty into the constraints.
3. **Model mismatch:** separate controller and plant models; add slip,
   hydrodynamic drag, current, delay, and actuator dynamics.
4. **Ablation evidence:** compare point-only, MRT-only, ROV-only, and full-body
   envelopes; compare static-position and velocity-aware obstacle prediction.
5. **Statistical validation:** run seeded randomized scenarios and report
   success, collision, minimum-clearance, tracking-error, and compute-time distributions.
6. **Higher-fidelity validation:** move from the kinematic node to a physics
   simulator and then to hardware-in-the-loop or a scale model.

For a paper-oriented continuation, the first three items should define the
method contribution, while the last three provide the evidence needed to
support it.
