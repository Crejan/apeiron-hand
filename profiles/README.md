# Hand profiles

`apeiron checkup` and `apeiron tune --save-profile` write `<serial>.json` here: per-joint gains
(`kp`, `kd`), an optional current limit (`max_effort`, A) and an optional preload
(`effort_bias`, A). The CLI and the adapters load the profile of the connected hand
automatically (`--profiles-dir`, `$APEIRON_PROFILES`, `./profiles`, `../profiles`).
Profiles are hardware specific and are not committed to this repository.
