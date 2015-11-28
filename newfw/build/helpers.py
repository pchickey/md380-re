
from ninja_syntax import Writer, as_list
from os.path import splitext
from os import system

def ninja_build(buildables):
    with open("build.ninja", "w") as buildfile:
        ninja = Writer(buildfile)
        basic_rules(ninja)
        for b in buildables:
            b.build(ninja)

    system("ninja")

def basic_rules(n):
    n.variable("cflags", "-g3 -Wall -std=gnu99 -Os -mlittle-endian -mthumb -mcpu=cortex-m4 -mfloat-abi=hard -mfpu=fpv4-sp-d16")

    n.variable("ldflags", "-mlittle-endian -mthumb -mcpu=cortex-m4 -mfloat-abi=hard -mfpu=fpv4-sp-d16")

    n.rule("compile",
            command="arm-none-eabi-gcc $cflags $includes -c $in -o $out",
            description="CC $out")

    n.rule("link",
            command="arm-none-eabi-gcc $ldflags -Wl,--script=$linkerscript -o $out $in $libs",
            description="LINK $out")

    n.rule("binary",
            command="arm-none-eabi-objcopy -O binary $in $out",
            description="OBJCOPY $out")


def change_extension(f, e):
    (base, orige) = splitext(f)
    return base + "." + e

class C_library(object):
    def __init__(self, path, sources=None, dependencies=None, extra_cflags=None):
        self.path = path
        if sources:
            self.sources = as_list(sources)
        else:
            self.sources = []

        if dependencies:
            self.dependencies = as_list(dependencies)
        else:
            self.dependencies = []

        if extra_cflags:
            self.extra_cflags = as_list(extra_cflags)
        else:
            self.extra_cflags = []

        self.built = False

    def collect_dependencies(self):
        ds = self.dependencies;
        for d in ds:
            ds += d.dependencies
        return ds

    def local(self, p):
        return self.path + "/" + p

    def include_path(self):
        ownpath = [ "-I./" + self.path ]
        deppath = []
        for d in self.collect_dependencies():
            deppath += d.include_path()
        return ownpath + deppath


    def cflags(self):
        cf = self.extra_cflags
        for d in self.collect_dependencies():
            cf += d.extra_cflags
        return cf

    def build(self, n):
        if self.built: return
        dep_paths = [ d.include_path() for d in self.collect_dependencies() ]
        for s in self.sources:
            vs = dict({ "includes": " ".join(self.include_path()),
                        "cflags": "$cflags " + " ".join(self.cflags()) })
            n.build(self.local(change_extension(s, "o")),
                    "compile",
                    inputs=self.local(s),
                    variables = vs)
        self.built = True

    def outputs(self, recursive = False):
        os = [ self.local(change_extension(s, "o")) for s in self.sources ]
        if recursive:
            for d in self.dependencies:
                os += d.outputs()
        return os


#########

boot_lib = C_library("boot",
                     sources=[ "system_stm32f4xx.c",
                               "startup_stm32f4xx.s",
                               "syscalls.c" ],
                     extra_cflags="-DHSE_VALUE=8000000")

class STM32F4App(object):
    def __init__(self, name, path=None, sources=None, dependencies=None):
        self.name = name
        if path:
            self.path = path
        else:
            self.path = name
        if sources:
            self.sources = as_list(sources)
        else:
            self.sources = []
        if dependencies:
            self.dependencies = as_list(dependencies)
        else:
            self.dependencies = []

    def linker_script(self):
        return "boot/stm32f405_flash_app.lds"

    def local(self, p):
        return self.path + "/" + p

    def collect_dependencies(self):
        ds = self.dependencies;
        for d in ds:
            ds += d.dependencies
        return ds

    def build(self, n):

        dep_outputs = []
        dep_includes = []
        dep_cflags   = []
        for d in self.collect_dependencies():
            d.build(n)
            dep_outputs += d.outputs()
            dep_includes += d.include_path()
            dep_cflags += d.cflags()

        includes = " ".join([("-I./" + self.path)] + dep_includes)
        cflags   = " ".join(dep_cflags)

        for s in self.sources:
            n.build(self.local(change_extension(s,"o")),
                    "compile",
                    inputs=self.local(s),
                    variables={"includes":includes,
                               "cflags": "$cflags " + cflags })

        n.build(self.local(self.name + ".elf"),
                "link",
                inputs= [ self.local(change_extension(s,"o")) for s in self.sources ]
                      + dep_outputs,
                variables= {"linkerscript": self.linker_script() })

        n.build(self.local(self.name + ".bin"),
                "binary",
                inputs=self.local(self.name + ".elf"))

class STM32F4Bootloader(STM32F4App):
    def linker_script(self):
        return "boot/stm32f405_flash_bl.lds"
