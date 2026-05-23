# Testing

Run the host-native suite:

```bash
scripts/test.sh
```

The suite compiles the `lib/xlrs` sources with native fallbacks selected, so the
link logic runs against a simulated clock, in-RAM flash, and a `MockPhy`
two-node radio sim.

The native suite covers:

- Link UID determinism and FHSS behavior.
- OTA packing/unpacking.
- AEAD crypto vectors and tamper rejection.
- PFD timing math.
- Link connect/failsafe/recovery flows.
- Rate switching and dynamic power logic.
- Corrupt RC rejection.
- Telemetry transport simulation.
- PHY recovery simulation.

Run clang-tidy for C++ logic changes:

```bash
scripts/lint.sh
```

Hardware behavior that mocks cannot prove should be captured in bench notes or
the active planning area for the work.
