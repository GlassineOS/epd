# GlassineOS/epd Constitution

This repository follows the GlassineOS organization principles (I–VIII). They come from
the `glassine` Spec Kit preset in `GlassineOS/.github` and are not edited per
repository. Repository-specific principles go in the last section.

## Organization principles

### I. Honest framing

GlassineOS is a hobby project, written largely with AI coding agents and not reviewed by
subsystem experts. Every README says so. Pull requests and commit messages state what was
verified and how (host test, emulator, which device and date), and say "untested" where
it applies.

### II. Safety before features

Anything that flashes, partitions, touches the bootloader, recovery, updates or user data
states its failure mode and its escape route before it is implemented. If the escape route
depends on what the change could break, the change needs explicit maintainer approval.
Device-affecting steps never run without the maintainer's go.

### III. Nothing vendor-owned, nothing personal

No vendor firmware or binaries, device dumps, decompiler output, keys, serial numbers, MAC
addresses or local paths in any repository. Firmware comes from the user's own device at
install time. The safety scan must pass.

### IV. Contracts are owned and versioned

Every interface between repositories (protocols, file formats, configuration, D-Bus or
socket APIs) has one owning repository that specifies and versions it. Consumers pin the
version they support and adopt new versions through their own spec. Breaking changes
bump the major version.

### V. Test at the right level

Libraries and daemons test their own behaviour in their own repository (host tests, no
device needed where possible). End-to-end behaviour of the assembled system is specified
and tested only as product acceptance in the `images` repository, as high-level
behavioural scenarios on hardware, never repeating library unit tests.

### VI. Device-neutral by default

No device constants outside device packages and device configuration. Code reads device
facts from configuration so a second device needs no changes outside its own packages.

### VII. E-ink first

Every user-facing change considers refresh cost, ghosting and latency on e-ink. Animations
and continuous redraws need a reason. Latency or power trade-offs are checked by feel on
the device, not by benchmark alone.

### VIII. Upstream and licenses

Our code is GPL-2.0-or-later unless the repository says otherwise; fork commits use the
upstream license. Nothing is submitted upstream unsolicited. AI-assisted commits carry an
`Assisted-by:` or `Co-Authored-By:` trailer; agents never add `Signed-off-by`.

## Repository principles

### Contracts

- **Owned:** the frame protocol (`src/glassine-epd-proto.h`), the output protocol
  (`src/glassine-epd-output-proto.h`), the adapter config (`/etc/glassine/epd.conf`), the
  refresh profiles (`/etc/glassine/eink.conf`), and the sockets, control FIFO and status
  files listed under "Runtime interfaces" in the README. The protocols carry a version;
  a change an old peer would misread raises it, and renaming a runtime name changes its
  consumers in the same release.
- **Consumed:** the kernel's DRM scanout of the panel, the panel's waveform file (read
  from the device at runtime, never stored here), and the device package's adapter
  config.

### Safety

Every fast path ends with the panel model matching what the panel shows. A path that can leave the panel or the GPU stuck has a recovery that
hands the panel back to the CPU engine.

### Tests and hardware

Host tests use the synthetic waveform. A new code path gets a test comparing it bit for
bit with the path it replaces (CPU reference against GPU, fast path against plain).
Latency and refresh changes are checked on a device by eye before they become a default,
and nothing is called hardware-proven without a recorded run.

## Governance

The organization principles change only in `GlassineOS/.github` (preset version bump);
repositories pick changes up through their `.glassine/org` submodule. Repository
principles change by pull request in this repository. Cross-repository decisions are
recorded as decision records in `GlassineOS/docs`.

Only `spec.md` is committed. It states what a feature does and why, in words that stay
true whatever the code looks like: no languages, modules, routes or other parts of the
implementation. Plans, research, data models, contracts drafted for a plan, and task
lists are working notes for one build. They are not committed, because the code changes
under them and they go stale; pull requests and commit messages record how something was
built and why. An interface other repositories depend on is the exception: it is
specified and versioned by its owner (principle IV).

**Version**: 1.0.0 | **Ratified**: 2026-11-13 | **Last Amended**: 2026-11-13
