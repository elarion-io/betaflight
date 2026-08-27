# Elarion flight controller firmware

This is a fork of [Betaflight](https://github.com/betaflight/betaflight) carrying
hardware support for Elarion flight controllers. Everything needed to build
firmware for Elarion boards lives in this one repository — target configurations
included, under [`elarion/configs/`](elarion/configs).

## Targets

| Target | MCU | Blackbox | Notes |
| --- | --- | --- | --- |
| `ELARIONH743` | STM32H743 | SPI flash (GD25Q128) | Default H7 build |
| `ELARIONH743SD` | STM32H743 | SD card (SPI) | Same board, SD card fitted instead |
| `ELARIONF405` | STM32F405 | — | |

The two H743 targets are the *same hardware*. The onboard flash chip and the SD
card socket share the SPI3 chip select on PC11, so only one can be used at a
time and each needs its own firmware image. Pick the one matching how your board
is populated; `status` in the CLI reports which image is running.

Both H743 images autodetect the barometer, so a single image covers boards fitted
with either the BMP581 or the LPS22BH.

## Flashing prebuilt firmware

Download the firmware for your target from the
[Releases](../../releases) page. Each release contains a `.hex` and a `.bin` per
target.

**Betaflight Configurator, or app.betaflight.com in a Chromium-based browser** —
use the `.hex`:

1. Put the board into DFU (bootloader) mode.
2. Firmware Flasher → **Load Firmware [Local]** → select the `.hex`.
3. Tick **Full chip erase**, then Flash.

**STM32CubeProgrammer or ST-Link** — use the `.bin`, flashed at address
`0x08000000`.

Either route erases your saved settings, so run `diff all` in the CLI first and
keep the output if you want to restore them afterwards.

## Building from source

You need a Linux, macOS or WSL environment with `make`, `git` and `curl`. A plain
clone is enough — no submodules are required for the Elarion targets:

```sh
git clone https://github.com/norduva/betaflight.git
cd betaflight
make arm_sdk_install     # first time only, downloads the ARM toolchain into tools/
make ELARIONH743
```

The firmware lands in `obj/` as
`betaflight_<version>_STM32H743_ELARIONH743.hex`. To also produce the raw binary
for CubeProgrammer:

```sh
make CONFIG=ELARIONH743 binary
```

Substitute `ELARIONH743SD` or `ELARIONF405` to build the other targets.

Two things worth knowing:

- **You do not need `make configs`.** That command hydrates upstream
  Betaflight's separate config repository, which this fork does not use — the
  Elarion configs are tracked here instead. `mk/local.mk` points `CONFIG_DIR` at
  `elarion/`, which is why bare `make ELARIONH743` works with no flags.
- Build with `make ELARIONH743 -j4` to use several cores. Avoid `-j$(nproc)` on
  machines with limited RAM; a full target is several hundred translation units.

## What differs from upstream Betaflight

- BMP581 barometer driver (also detects the register-compatible BMP580)
- LPS22BH barometer driver
- GD25Q128 SPI flash support
- MAX7456 OSD SPI mode fix
- Elarion target configurations

## Licence

Betaflight is licensed under the GNU General Public License v3. This fork is
distributed under the same terms; see [LICENSE](LICENSE). If you received
binaries from us, the corresponding source is this repository.
