# Copyright © 2018, VideoLAN and dav1d authors
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

# Dependencies
libm_dependency     = cc.find_library('m', required: false)
rt_dependency       = cc.find_library('rt', required: false)
libdl_dependency    = cc.find_library('dl', required: false)
thread_dependency   = dependency('threads')

have_pthread_np = cc.check_header('pthread_np.h')
if have_pthread_np
  test_args += '-DHAVE_PTHREAD_NP_H'
endif

have_pthread_setaffinity_np = cc.has_function('pthread_setaffinity_np',
  prefix: '''
    #include <pthread.h>
    #ifdef HAVE_PTHREAD_NP_H
    #include <pthread_np.h>
    #endif
  ''',
  args: test_args,
  dependencies: thread_dependency,
)

have_getauxval = false
have_elf_aux_info = false
if (arch_aarch64 or arch_arm or arch_loongarch or arch_ppc64le or arch_riscv)
    have_getauxval = cc.has_function('getauxval', prefix : '#include <sys/auxv.h>', args : test_args)
    have_elf_aux_info = cc.has_function('elf_aux_info', prefix : '#include <sys/auxv.h>', args : test_args)
endif

use_gaspp = false
if ((arch_aarch64 or arch_arm) and
     cc.get_argument_syntax() == 'msvc' and cc.get_id() != 'clang-cl')
    gaspp = find_program('gas-preprocessor.pl')
    use_gaspp = true
    gaspp_args = [
            '-as-type', 'armasm',
            '-arch', host_machine.cpu_family(),
            '--',
            arch_aarch64 ? 'armasm64' : 'armasm',
            '-nologo',
            '-I@0@'.format(meson.current_source_dir()),
            '-I@0@'.format(meson.current_build_dir()),
    ]
    gaspp_gen = generator(gaspp,
        output: '@BASENAME@.obj',
        arguments: gaspp_args + [
            '@INPUT@',
            '-c',
            '-o', '@OUTPUT@'
        ])
endif

if host_machine.system() == 'windows'
    if get_option('default_library') != 'static'
        api_export_flags = ['-DCHECKASM_BUILDING_DLL']
    endif
endif

have_linux_perf = false
if host_machine.system() == 'linux'
  have_linux_perf = cc.has_header_symbol('linux/perf_event.h', 'PERF_COUNT_HW_CPU_CYCLES', args: test_args)
endif

# Only depends on <dlfcn.h>, and not needed on other architectures.
have_macos_kperf = host_machine.system() == 'darwin' and arch_aarch64

# Build configuration
cdata = configuration_data()
cdata.set('PREFIX',                         cc.symbols_have_underscore_prefix())
cdata.set10('ARCH_AARCH64',                 arch_aarch64)
cdata.set10('ARCH_ARM',                     arch_arm)
cdata.set10('ARCH_X86',                     arch_x86)
cdata.set10('ARCH_X86_64',                  arch_x86_64)
cdata.set10('ARCH_X86_32',                  arch_x86_32)
cdata.set10('ARCH_PPC64LE',                 arch_ppc64le)
cdata.set10('ARCH_RISCV',                   arch_riscv)
cdata.set10('ARCH_RV32',                    arch_rv32)
cdata.set10('ARCH_RV64',                    arch_rv64)
cdata.set10('ARCH_LOONGARCH',               arch_loongarch)
cdata.set10('ARCH_LOONGARCH32',             arch_loongarch32)
cdata.set10('ARCH_LOONGARCH64',             arch_loongarch64)
cdata.set10('HAVE_UNISTD_H',                cc.check_header('unistd.h'))
cdata.set10('HAVE_PTHREAD_NP_H',            have_pthread_np)
cdata.set10('HAVE_PTHREAD_SETAFFINITY_NP',  have_pthread_setaffinity_np)
cdata.set10('HAVE_GETAUXVAL',               have_getauxval)
cdata.set10('HAVE_ELF_AUX_INFO',            have_elf_aux_info)
cdata.set10('HAVE_LINUX_PERF',              have_linux_perf)
cdata.set10('HAVE_MACOS_KPERF',             have_macos_kperf)

if arch_x86
  cdata.set10('PIC', true)
  cdata.set10('FORCE_VEX_ENCODING', cc.get_define('__AVX__').strip() != '')
endif

checkasm_config = []
checkasm_config += configure_file(
  output: 'config.h',
  configuration: cdata,
  install: false,
)

# Build definition
checkasm_asm_objs = []
checkasm_sources = files(
  'checkasm.c',
  'cpu.c',
  'perf.c',
  'signal.c',
  'stats.c',
  'utils.c',
)

if have_linux_perf
  checkasm_sources += files('perf/linux.c')
endif

if have_macos_kperf
  checkasm_sources += files('perf/macos_kperf.c')
endif

if arch_aarch64
  checkasm_asm_sources = files('arm/checkasm_64.S')
  if use_gaspp
    checkasm_asm_objs += gaspp_gen.process(checkasm_asm_sources)
  else
    checkasm_sources += checkasm_asm_sources
  endif
elif arch_arm
  checkasm_asm_sources = files('arm/checkasm_32.S')
  if use_gaspp
    checkasm_asm_objs += gaspp_gen.process(checkasm_asm_sources)
  else
    checkasm_sources += checkasm_asm_sources
  endif
elif arch_rv64
  checkasm_sources += files('riscv/checkasm_64.S')
elif arch_loongarch
  checkasm_sources += files('loongarch/checkasm.S')
elif arch_x86
  add_languages('nasm', native: false)
  configure_file(
    output: 'checkasm_config.asm',
    output_format: 'nasm',
    configuration: cdata,
    install: false,
  )
  checkasm_sources += files(
    'x86/checkasm.asm',
    'x86/cpu.c',
  )
endif

checkasm_dependencies = [
  thread_dependency,
  rt_dependency,
  libdl_dependency,
  libm_dependency,
]

checkasm_library = library(meson.project_name(),
  checkasm_sources,
  checkasm_config,
  checkasm_asm_objs,
  include_directories: checkasm_inc_dirs,
  dependencies: checkasm_dependencies,
  install: true,
)

checkasm_dep = declare_dependency(
  link_with: checkasm_library,
  include_directories: checkasm_inc_dirs,
)

pkg = import('pkgconfig')
pkg.generate(
  name: meson.project_name(),
  description: 'checkasm for all your asm checking needs',
  libraries: checkasm_library,
  version: meson.project_version(),
)
