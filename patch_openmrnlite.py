"""PlatformIO pre-build extra_script.

OpenMRNLite v2.0.0's vendored Esp32Gpio.hxx has a stray ')' in the Octal-SPI
static_assert around line 97, which breaks any ESP32-S3 build:

    static_assert(!(PIN_NUM >= 33 && PIN_NUM <= 37)),
                  "Pin is not available when Octal SPI mode is enabled.");

(the closing ')' after the boolean expression prematurely ends
static_assert(...), leaving a dangling ", "message");" as garbage tokens).

.pio/libdeps is git-ignored, so this fixes the freshly-installed libdep in
place, idempotently, every time PlatformIO builds -- instead of relying on a
manual edit that a clean checkout/CI would silently lose.
"""
import glob
import os

Import("env")  # noqa: F821 -- injected by PlatformIO/SCons

BROKEN = "static_assert(!(PIN_NUM >= 33 && PIN_NUM <= 37)),"
FIXED = "static_assert(!(PIN_NUM >= 33 && PIN_NUM <= 37),"


def patch_esp32_gpio():
    project_dir = env.subst("$PROJECT_DIR")  # noqa: F821
    pattern = os.path.join(project_dir, ".pio", "libdeps", "*", "OpenMRNLite",
        "src", "freertos_drivers", "esp32", "Esp32Gpio.hxx")
    matches = glob.glob(pattern)
    if not matches:
        print("[patch_openmrnlite] Esp32Gpio.hxx not found (lib not "
              "installed yet); nothing to patch")
        return

    for path in matches:
        with open(path, "r") as f:
            text = f.read()

        if FIXED in text:
            print("[patch_openmrnlite] already patched: %s" % path)
            continue

        if BROKEN not in text:
            print("[patch_openmrnlite] WARNING: expected static_assert "
                  "text not found, leaving untouched: %s" % path)
            continue

        text = text.replace(BROKEN, FIXED)
        with open(path, "w") as f:
            f.write(text)
        print("[patch_openmrnlite] patched stray ')' in %s" % path)


patch_esp32_gpio()
