# All-pitch-down reference

N=10; seven locked phase offsets defined on the ten-segment reference. Body-yaw half-amplitude is 19 deg, the stronger moderate PD is unchanged, and leg yaw is fixed. Every leg remains commanded down after settling. This is deterministic, not IID, and ground contact is not artificially forced.

Each reference has four warm-up cycles and twelve evaluated cycles. The CSV records signed COM displacement and realized contact fraction; the videos show the full evaluation.

The IID overlay is descriptive only: the previous IID sweep omitted its MuJoCo version, while these references explicitly use 3.3.7.
