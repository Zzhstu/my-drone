# Frozen offline coverage reference

`mesh_reference.npz` and `target_manifest.json` contain the existing known-scene CAD surface audit targets for `cangku_20sqm.world`, not old measured maps, images or camera trajectories. The reference has 4,407 externally observable 20 cm surface targets; 1,430 buried/occluded/domain-unobservable targets remain in its exclusion mask. The physical drone capability envelope and the same two-image/40 cm baseline test remain frozen.

The NPZ SHA256 is `fa56cfe7edd93370012d1e15e8f8e53858d69d134b288474b2a5c08bba062d67`. The manifest ties it to the warehouse world and rack mesh SHA256. Geometry is used only for offline coverage auditing. It must never supply points or poses to the measured dataset.

Regenerate the reference for an intentionally changed world with `integration/roomgs/warehouse20_audit.py --prepare OUTPUT` in the roomgs environment, and explicitly rebuild/revalidate the matching inspection plan. Do not change the target denominator to make an incomplete capture pass.
