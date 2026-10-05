# Frozen rigid-body sampled-PD check

Read-only model. No controller, physics, gains, contacts or historical results were modified.

URDF SHA: `c1d454f479246a095e9f90597213674c4abc28a7661c0573dca78e43bd0e7c7f`; total mass 16.512 kg. Full generalized velocity order is world base linear xyz, world base angular xyz, then the explicitly mapped 12 Adapter joint names.

The 18DOF matrix sums each link COM translational and rotated inertial angular kinetic energy. Base-fixed and force-free floating Schur models are different idealized boundaries, neither is the four-contact robot.

| Snapshot | Boundary | Minimum inertia kg m² | P220.982919/D1 rho | Pole margin | Strict D interval |
|---|---|---:|---:|---:|---|
| Adapter nominal | fixed_base | 0.003655677 | 0.951202 | 0.048798 | (0.321377, 1.714108) |
| Adapter nominal | floating_base_Schur | 0.003463728 | 0.949751 | 0.050249 | (0.321375, 1.618092) |
| actual JTC feedback 100.000s | fixed_base | 0.003739489 | 0.949977 | 0.050023 | (0.321375, 1.756030) |
| actual JTC feedback 100.000s | floating_base_Schur | 0.003626152 | 0.947586 | 0.052414 | (0.321370, 1.699339) |

The model assumes 4ms frozen PD force and four 1ms semi-implicit Euler steps, with URDF viscous damping .01. It also reports the no-passive and exact inertial 4ms ZOH checks. All four P221/D1 cases have rho < 1 in each of these models. P221/D3.360637 has a real negative unstable pole in all four boundaries. P100/D1 and the unverified old-comment P67.5/D.3 are included as numerical comparisons only.

Nine numerical checks pass, including all 18 COM and world-angular Jacobian columns by centered finite differences for both snapshots; maximum Jacobian residual is 1.10e-10 and mass-matrix residual 2.83e-11. Independent review uses another FK/step implementation.

The actual snapshot is read from the preserved 100s JTC data using joint names, not the measured-sensor array order. Maximum q difference from Adapter nominal is 0.15503915045776462 rad. This is only one measured posture, not a workspace robustness certificate.

No Coulomb friction .2, foot constraints, impacts, gravity Hessian, rotor armature, command/velocity saturation, measurement delay, integrator I=.2, target interpolation or reference-effort carry is modeled. Original JTC=false contains stateful FF carry, so an ideal P100/D1 spectral result is not a stability proof for the original controller. No computed D is adopted; a new native JTC contract may only establish real parameter/command semantics.

Reproduce without ROS: `python3 multifloor_demo/navigation/test_results/rigid_body_pd_model/model.py`. Exact sources, matrices and all model values are in result.json and the two NPZ files.
