# Bundled GTN runtime files

These files were copied from the user-provided `QT_REFACTOR_V1` runtime on
2026-08-12 so that the SDK has a self-contained default hardware path.

| File | Source | SHA-256 |
|---|---|---|
| `Gecat_15axis.eni` | `QT_REFACTOR_V1/config/Gecat.eni` | `56b330b1894673ac50670c78d45e46d548a2129a84e693586e25a849e2ab0a80` |
| `gtn_core1.cfg` | `QT_REFACTOR_V1/config/gtn_core1.cfg` | `7ca12fbd17bb8cd1cfcfd17944b3a3b77848d468a15b631e3872db6ab37f7b78` |
| `libgxn_x64.so` | `QT_REFACTOR_V1/lib/gxn/libgxn_x64.so` | `79db2b5ea558b9b0f32d2ef902bf394d168a01a8ad4545ad1988b8cbae271cb8` |

`Gecat_15axis.eni` declares `slavenum=15` and is the default only for
`coffee_15_no_slide`. A future 16-axis machine must supply a matching 16-axis
ENI file and must not reuse this file.

The dynamic library is a vendor binary. Confirm its redistribution and runtime
license with the hardware vendor before distributing the SDK outside the
authorized deployment environment.
