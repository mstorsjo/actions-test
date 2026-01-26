# Copyright © 2018-2019, VideoLAN and dav2d authors
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
# ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#
# Build definition for the dav2d library
#

# libdav2d source files
libdav2d_sources = files(
    'cdf.c',
    'cpu.c',
    'ctx.c',
    'data.c',
    'decode.c',
    'dip_tables.c',
    'getbits.c',
    'ibp.c',
    'itx_1d.c',
    'lf_mask.c',
    'lib.c',
    'log.c',
    'mem.c',
    'msac.c',
    'obu.c',
    'pal.c',
    'picture.c',
    'qm.c',
    'ref.c',
    'refmvs.c',
    'scan.c',
    'stx_tables.c',
    'tables.c',
    'thread_task.c',
    'warpmv.c',
    'wedge.c',
)

# libdav2d bitdepth source files
# These files are compiled for each bitdepth with
# `BITDEPTH` defined to the currently built bitdepth.
libdav2d_tmpl_sources = files(
    'ccso_tmpl.c',
    'cdef_apply_tmpl.c',
    'cdef_tmpl.c',
    'fg_apply_tmpl.c',
    'filmgrain_tmpl.c',
    'ipred_prepare_tmpl.c',
    'ipred_tmpl.c',
    'itx_tmpl.c',
    'lf_apply_tmpl.c',
    'loopfilter_tmpl.c',
    'looprestoration_tmpl.c',
    'lr_apply_tmpl.c',
    'mc_tmpl.c',
    'recon_tmpl.c',
    'stx_tmpl.c',
)

libdav2d_arch_tmpl_sources = {}

libdav2d_bitdepth_objs = []

# ASM specific sources
libdav2d_asm_objs = []
# Arch-specific flags
arch_flags = {}
if is_asm_enabled
    if (host_machine.cpu_family() == 'aarch64' or
        host_machine.cpu_family().startswith('arm'))

        libdav2d_sources += files(
            'arm/cpu.c',
        )
        if (host_machine.cpu_family() == 'aarch64' or
            host_machine.cpu() == 'arm64')
            libdav2d_sources_asm = files(
                # itx.S is used for both 8 and 16 bpc.
                'arm/64/itx.S',
                'arm/64/looprestoration_common.S',
                'arm/64/msac.S',
                'arm/64/refmvs.S',
            )

            if dav2d_bitdepths.contains('8')
                libdav2d_sources_asm += files(
                    'arm/64/cdef.S',
                    'arm/64/filmgrain.S',
                    'arm/64/ipred.S',
                    'arm/64/loopfilter.S',
                    'arm/64/looprestoration.S',
                    'arm/64/mc.S',
                    'arm/64/mc_dotprod.S',
                )
            endif

            if dav2d_bitdepths.contains('16')
                libdav2d_sources_asm += files(
                    'arm/64/cdef16.S',
                    'arm/64/filmgrain16.S',
                    'arm/64/ipred16.S',
                    'arm/64/itx16.S',
                    'arm/64/loopfilter16.S',
                    'arm/64/looprestoration16.S',
                    'arm/64/mc16.S',
                    'arm/64/mc16_sve.S',
                )
            endif
        elif host_machine.cpu_family().startswith('arm')
            libdav2d_sources_asm = files(
                # itx.S is used for both 8 and 16 bpc.
                'arm/32/itx.S',
                'arm/32/looprestoration_common.S',
                'arm/32/msac.S',
                'arm/32/refmvs.S',
            )

            if dav2d_bitdepths.contains('8')
                libdav2d_sources_asm += files(
                    'arm/32/cdef.S',
                    'arm/32/filmgrain.S',
                    'arm/32/ipred.S',
                    'arm/32/loopfilter.S',
                    'arm/32/looprestoration.S',
                    'arm/32/mc.S',
                )
            endif

            if dav2d_bitdepths.contains('16')
                libdav2d_sources_asm += files(
                    'arm/32/cdef16.S',
                    'arm/32/filmgrain16.S',
                    'arm/32/ipred16.S',
                    'arm/32/itx16.S',
                    'arm/32/loopfilter16.S',
                    'arm/32/looprestoration16.S',
                    'arm/32/mc16.S',
                )
            endif
        endif

        if use_gaspp
            libdav2d_asm_objs = gaspp_gen.process(libdav2d_sources_asm)
        else
            libdav2d_sources += libdav2d_sources_asm
        endif
    elif host_machine.cpu_family().startswith('x86')

        libdav2d_sources += files(
            'x86/cpu.c',
        )

        # NASM source files
        libdav2d_sources_asm = files(
            'x86/cpuid.asm',
            'x86/msac.asm',
            'x86/pal.asm',
            'x86/refmvs.asm',
            'x86/itx_avx512.asm',
            'x86/cdef_avx2.asm',
            'x86/itx_avx2.asm',
            'x86/cdef_sse.asm',
            'x86/itx_sse.asm',
        )

        if dav2d_bitdepths.contains('8')
            libdav2d_sources_asm += files(
                'x86/cdef_avx512.asm',
                'x86/filmgrain_avx512.asm',
                'x86/ipred_avx512.asm',
                'x86/loopfilter_avx512.asm',
                'x86/looprestoration_avx512.asm',
                'x86/mc_avx512.asm',
                'x86/filmgrain_avx2.asm',
                'x86/ipred_avx2.asm',
                'x86/loopfilter_avx2.asm',
                'x86/looprestoration_avx2.asm',
                'x86/mc_avx2.asm',
                'x86/filmgrain_sse.asm',
                'x86/ipred_sse.asm',
                'x86/loopfilter_sse.asm',
                'x86/looprestoration_sse.asm',
                'x86/mc_sse.asm',
            )
        endif

        if dav2d_bitdepths.contains('16')
            libdav2d_sources_asm += files(
                'x86/cdef16_avx512.asm',
                'x86/filmgrain16_avx512.asm',
                'x86/ipred16_avx512.asm',
                'x86/itx16_avx512.asm',
                'x86/loopfilter16_avx512.asm',
                'x86/looprestoration16_avx512.asm',
                'x86/mc16_avx512.asm',
                'x86/cdef16_avx2.asm',
                'x86/filmgrain16_avx2.asm',
                'x86/ipred16_avx2.asm',
                'x86/itx16_avx2.asm',
                'x86/loopfilter16_avx2.asm',
                'x86/looprestoration16_avx2.asm',
                'x86/mc16_avx2.asm',
                'x86/cdef16_sse.asm',
                'x86/filmgrain16_sse.asm',
                'x86/ipred16_sse.asm',
                'x86/itx16_sse.asm',
                'x86/loopfilter16_sse.asm',
                'x86/looprestoration16_sse.asm',
                'x86/mc16_sse.asm',
            )
        endif

        # Compile the ASM sources with NASM
        libdav2d_asm_objs = nasm_gen.process(libdav2d_sources_asm)
    elif host_machine.cpu_family().startswith('loongarch')
        libdav2d_sources += files(
            'loongarch/cpu.c',
        )

        libdav2d_arch_tmpl_sources += {'default': files(
            'loongarch/looprestoration_inner.c',
        )}

        libdav2d_sources_asm = files(
            'loongarch/cdef.S',
            'loongarch/ipred.S',
            'loongarch/mc.S',
            'loongarch/loopfilter.S',
            'loongarch/looprestoration.S',
            'loongarch/msac.S',
            'loongarch/refmvs.S',
            'loongarch/itx.S',
        )
        libdav2d_asm_objs += libdav2d_sources_asm
    elif host_machine.cpu() == 'ppc64le'
        arch_flags += {'vsx': ['-maltivec', '-mvsx', '-DDAV2D_VSX']}
        libdav2d_sources += files(
            'ppc/cpu.c',
        )
        libdav2d_arch_tmpl_sources += {'vsx': files(
            'ppc/cdef_tmpl.c',
            'ppc/looprestoration_tmpl.c',
        )}
        arch_flags += {'pwr9': ['-mcpu=power9', '-DDAV2D_PWR9']}
        libdav2d_arch_tmpl_sources += {'pwr9': files(
            'ppc/itx_tmpl.c',
            'ppc/loopfilter_tmpl.c',
            'ppc/mc_tmpl.c',
        )}
    elif host_machine.cpu_family().startswith('riscv')
        libdav2d_sources += files(
            'riscv/cpu.c',
        )
        if host_machine.cpu_family() == 'riscv64'
            libdav2d_sources += files(
                'riscv/64/cpu.S',
                'riscv/64/pal.S',
            )

            if dav2d_bitdepths.contains('8')
                libdav2d_sources += files(
                    'riscv/64/cdef.S',
                    'riscv/64/ipred.S',
                    'riscv/64/itx.S',
                    'riscv/64/mc.S',
                )
            endif

            if dav2d_bitdepths.contains('16')
                libdav2d_sources += files(
                    'riscv/64/cdef16.S',
                    'riscv/64/ipred16.S',
                    'riscv/64/mc16.S',
                )
            endif
        endif
    endif
endif



libdav2d_rc_obj = []
libdav2d_flags = []
api_export_flags = []

#
# Windows .rc file and API export flags
#

if host_machine.system() == 'windows'
    if get_option('default_library') != 'static'
        rc_file = configure_file(
            input : 'dav2d.rc.in',
            output : 'dav2d.rc',
            configuration : rc_data
        )

        libdav2d_rc_obj = winmod.compile_resources(rc_file)

        api_export_flags = ['-DDAV2D_BUILDING_DLL']
    endif

    if (host_machine.cpu_family() == 'x86_64' and cc.get_id() == 'gcc')
        # We don't expect to reference data members from other DLLs without
        # dllimport attributes. Set the -mcmodel=small flag, which avoids
        # generating indirection via .refptr.<symname> for all potentially
        # dllimported variable references.
        libdav2d_flags += '-mcmodel=small'
    endif
endif



#
# Library definitions
#

# Helper library for each bitdepth
libdav2d_bitdepth_objs = []
foreach bitdepth : dav2d_bitdepths
    libdav2d_bitdepth_objs += static_library(
        'dav2d_bitdepth_@0@'.format(bitdepth),
        libdav2d_tmpl_sources, config_h_target,
        include_directories: dav2d_inc_dirs,
        dependencies : [stdatomic_dependencies],
        c_args : ['-DBITDEPTH=@0@'.format(bitdepth)] + libdav2d_flags,
        install : false,
        build_by_default : false,
    ).extract_all_objects(recursive: true)
endforeach

# Helper library for each bitdepth and architecture-specific flags
foreach bitdepth : dav2d_bitdepths
    foreach subarch : libdav2d_arch_tmpl_sources.keys()
        libdav2d_bitdepth_objs += static_library(
            'dav2d_arch_bitdepth_@0@_@1@'.format(bitdepth,subarch),
            libdav2d_arch_tmpl_sources[subarch], config_h_target,
            include_directories: dav2d_inc_dirs,
            dependencies : [stdatomic_dependencies],
            c_args : ['-DBITDEPTH=@0@'.format(bitdepth)] + libdav2d_flags + arch_flags.get(subarch, []),
            install : false,
            build_by_default : false,
        ).extract_all_objects(recursive: true)
    endforeach
endforeach

# The final dav2d library
if host_machine.system() == 'windows'
    dav2d_soversion = ''
else
    dav2d_soversion = dav2d_api_version_major
endif

libdav2d = library('dav2d',
    libdav2d_sources,
    libdav2d_asm_objs,
    libdav2d_rc_obj,
    rev_target,
    config_h_target,

    objects : [
        libdav2d_bitdepth_objs,
        ],

    include_directories : dav2d_inc_dirs,
    dependencies : [
        stdatomic_dependencies,
        thread_dependency,
        thread_compat_dep,
        libdl_dependency,
        ],
    c_args : [libdav2d_flags, api_export_flags],
    version : dav2d_soname_version,
    soversion : dav2d_soversion,
    install : true,
)

dav2d_dep = declare_dependency(link_with: libdav2d,
    include_directories : include_directories('../include')
)

#
# Generate pkg-config .pc file
#
pkg_mod = import('pkgconfig')
pkg_mod.generate(libraries: libdav2d,
    version: meson.project_version(),
    name: 'libdav2d',
    filebase: 'dav2d',
    description: 'AV2 decoding library'
)
