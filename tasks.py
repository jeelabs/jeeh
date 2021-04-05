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

@task
def fail(c, addr):
    """lookup failAt address using addr2line"""
    env = "nucleo-l432"
    c.run("arm-none-eabi-addr2line -e .pio/build/%s/firmware.elf %s" % (env, addr));
