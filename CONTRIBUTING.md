# Contributing

Thanks for helping. A few ground rules keep the project small and trustworthy.

## Scope

Apeiron implements the Wuji Hand 2 protocol as observed on real devices. Anything added
to the specification must have been verified against a hand (state firmware and hardware
revision in the pull request), or be clearly marked as unverified in code and docs.

## Layering

`apeiron-protocol` → `apeiron-driver` → `apeiron-cli` / adapters. The protocol crate has
no I/O; the driver has no terminal output; the CLI has no Zenoh calls. Put changes where
`docs/crates.md` says they belong.

## Code

- `cargo fmt`, `cargo clippy --all-targets` and `cargo test` clean.
- Short doc comments on public items; explain *why* when the code cannot.
- Tests only where they prove behaviour, preferably against bytes captured from a device
  (see `apeiron-protocol/src/messages.rs`). No tests that merely restate an interface.
- Everything that can move the hand must stay behind an explicit opt-in (`--allow-motion`
  in the CLI, an explicit `arm`/`run`/`start` in the driver) and respect the current
  limit.

## Documentation

Docs live in `docs/` and are built with Zensical (`pip install zensical`,
`zensical serve`). Keep the specification factual and English; keep guides task-oriented.

## Reporting a firmware difference

Open an issue with `apeiron capabilities --summary`, `apeiron status`, the firmware
version and what differed. Capability dumps and short `apeiron record` files are the most
useful attachments.

## License

Contributions are accepted under MIT OR Apache-2.0, the project's license.
