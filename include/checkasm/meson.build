# Copyright © 2025 Niklas Haas
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

checkasm_api_headers = files(
  'attributes.h',
  'checkasm.h',
  'test.h',
  'utils.h',
)

# Platform-specific code and definitions (besides timing)
if arch_x86_64
  platform_header = files('platform/x86_64.h')
elif arch_x86_32
  platform_header = files('platform/x86_32.h')
elif arch_arm
  platform_header = files('platform/arm.h')
elif arch_aarch64
  platform_header = files('platform/aarch64.h')
elif arch_ppc64le
  platform_header = files('platform/ppc64le.h')
elif arch_riscv
  platform_header = files('platform/riscv.h')
elif arch_loongarch
  platform_header = files('platform/loongarch.h')
else
  platform_header = files('platform/generic.h')
endif

# Benchmarking / timing subsystem
if arch_x86
  perf_header = files('perf/x86.h')
elif arch_aarch64
  perf_header = files('perf/aarch64.h')
elif arch_arm
  perf_header = files('perf/arm.h')
elif arch_ppc64le
  perf_header = files('perf/ppc64le.h')
elif arch_loongarch64
  perf_header = files('perf/loongarch64.h')
elif arch_loongarch32
  perf_header = files('perf/loongarch32.h')
else
  perf_header = files('perf/dummy.h')
endif

# Setjmp/longjmp implementation
if host_machine.system() == 'windows'
  if arch_x86_32
    longjmp_header = 'longjmp/win32.h'
  else
    longjmp_header = 'longjmp/win64.h'
  endif
else
  longjmp_header = files('longjmp/posix.h')
endif

install_headers(checkasm_api_headers, subdir: meson.project_name())

configure_file(
  input: platform_header,
  output: 'platform.h',
  install: true,
  install_dir: get_option('includedir') / meson.project_name(),
  copy: true,
)

configure_file(
  input: perf_header,
  output: 'perf.h',
  install: true,
  install_dir: get_option('includedir') / meson.project_name(),
  copy: true,
)

configure_file(
  input: longjmp_header,
  output: 'longjmp.h',
  install: true,
  install_dir: get_option('includedir') / meson.project_name(),
  copy: true,
)
