from invoke import task

@task(default=True)
def run(c):
    """compile, upload, and connect"""
    c.run("pio run -t upload -t monitor", pty=True)

@task
def disas(c):
    """show code disassembly"""
    c.run("arm-none-eabi-objdump -dC .pio/build/nucleo-l432/firmware.elf")

@task
def symbols(c):
    """show a memory map with symbols"""
    c.run("arm-none-eabi-nm -gCnS .pio/build/nucleo-l432/firmware.elf |"
          "grep -v Handler")
