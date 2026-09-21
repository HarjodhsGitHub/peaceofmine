#!/usr/bin/env python3
"""Build only, never upload. Uses the vendored ArbotiX core and AVR GCC."""
import argparse
import os
from pathlib import Path
import shutil
import subprocess


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--toolchain', type=Path, help='Directory containing avr-g++')
    parser.add_argument('--output', type=Path, default=Path('/tmp/peaceofmine-safe-arm'))
    args = parser.parse_args()
    toolchain = args.toolchain
    if toolchain is None:
        gcc = shutil.which('avr-g++')
        toolchain = (Path(gcc).parent if gcc else
                     Path(os.environ.get('PLATFORMIO_CORE_DIR', Path.home() / '.platformio')) /
                     'packages/toolchain-atmelavr/bin')
    if not (toolchain / 'avr-g++').is_file():
        parser.error('AVR GCC not found; specify --toolchain or install the PlatformIO AVR toolchain')
    here = Path(__file__).resolve().parent
    upstream = here.parent / 'arbotix_ros'
    core = upstream / 'hardware/arbotix/cores/arbotix'
    variant = upstream / 'hardware/arbotix/variants/standard'
    library = upstream / 'libraries/Bioloid'
    args.output.mkdir(parents=True, exist_ok=True)
    common = ['-mmcu=atmega644p', '-DF_CPU=16000000L', '-DARDUINO=100', '-Os',
              '-ffunction-sections', '-fdata-sections', '-Wall',
              f'-I{core}', f'-I{variant}', f'-I{library}']
    sources = sorted(core.glob('*.c')) + sorted(core.glob('*.cpp'))
    sources += [library / 'ax12.cpp', here / 'safe_arm.ino']
    objects = []
    for index, source in enumerate(sources):
        obj = args.output / f'{index}-{source.stem}.o'
        cpp = source.suffix != '.c'
        compiler = toolchain / ('avr-g++' if cpp else 'avr-gcc')
        flags = ['-x', 'c++', '-std=gnu++11', '-fno-exceptions', '-fno-threadsafe-statics'] if cpp else []
        subprocess.run([str(compiler), *common, *flags, '-c', str(source), '-o', str(obj)], check=True)
        objects.append(str(obj))
    elf = args.output / 'safe_arm.elf'
    subprocess.run([str(toolchain / 'avr-g++'), '-mmcu=atmega644p', '-Wl,--gc-sections',
                    *objects, '-lm', '-o', str(elf)], check=True)
    subprocess.run([str(toolchain / 'avr-size'), '-C', '--mcu=atmega644p', str(elf)], check=True)
    subprocess.run([str(toolchain / 'avr-objcopy'), '-O', 'ihex', '-R', '.eeprom', str(elf),
                    str(args.output / 'safe_arm.hex')], check=True)
    print(f'Built {args.output / "safe_arm.hex"}; no hardware was accessed.')


if __name__ == '__main__':
    main()
