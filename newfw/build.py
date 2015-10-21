#!/usr/bin/env python

from os.path import splitext
from ninja_syntax import Writer, as_list

def basic_rules(n):
    n.variable("cflags", "-g3 -Wall -std=gnu99 -mlittle-endian -mthumb -mcpu=cortex-m4 -mfloat-abi=hard -mfpu=fpv4-sp-d16")

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


    def build(self, n):
        dep_paths = [ d.include_path() for d in self.collect_dependencies() ]
        for s in self.sources:
            vs = dict({ "includes": " ".join(self.include_path()),
                        "cflags": "$cflags " + " ".join(self.extra_cflags) })
            n.build(self.local(change_extension(s, "o")),
                    "compile",
                    inputs=self.local(s),
                    variables = vs)

    def outputs(self, recursive = False):
        os = [ self.local(change_extension(s, "o")) for s in self.sources ]
        if recursive:
            for d in self.dependencies:
                os += d.outputs()
        return os


#########

bare_metal_lib = C_library("bare_metal_build",
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
        return "bare_metal_build/stm32f405_flash.lds"

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
        for d in self.collect_dependencies():
            d.build(n)
            dep_outputs += d.outputs()
            dep_includes += d.include_path()

        includes = " ".join([("-I./" + self.path)] + dep_includes)

        for s in self.sources:
            n.build(self.local(change_extension(s,"o")),
                    "compile",
                    inputs=self.local(s),
                    variables={"includes":includes})

        n.build(self.local(self.name + ".elf"),
                "link",
                inputs= [ self.local(change_extension(s,"o")) for s in self.sources ]
                      + dep_outputs,
                variables= {"linkerscript": self.linker_script() })

        n.build(self.local(self.name + ".bin"),
                "binary",
                inputs=self.local(self.name + ".elf"))

#######


with open("build.ninja", "w") as buildfile:
    no_os_hwf4 = C_library("hwf4",
                    sources=[ "rcc.c", "gpio.c"],
                    dependencies=bare_metal_lib)
    blink = STM32F4App("blink", sources="blink.c", dependencies=no_os_hwf4)

    ninja = Writer(buildfile)

    basic_rules(ninja)
    blink.build(ninja)


