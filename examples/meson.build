# Copyright © 2018, VideoLAN and dav2d authors
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
# Build definition for the dav2d examples
#

# Leave subdir if examples are disabled
if not get_option('enable_examples')
    subdir_done()
endif


# dav2d player sources
dav2dplay_sources = files(
    'dav2dplay.c',
    'dp_fifo.c',
    'dp_renderer_placebo.c',
    'dp_renderer_sdl.c',
)

sdl2_dependency = dependency('sdl2', version: '>= 2.0.1', required: true)

if sdl2_dependency.found()
    dav2dplay_deps = [sdl2_dependency, libm_dependency]
    dav2dplay_cflags = []

    placebo_dependency = dependency('libplacebo', version: '>= 4.160.0', required: false)

    have_vulkan = false
    have_placebo = placebo_dependency.found()
    if have_placebo
        dav2dplay_deps += placebo_dependency

        # If libplacebo is found, we might be able to use Vulkan
        # with it, in which case we need the Vulkan library too.
        vulkan_dependency = dependency('vulkan', required: false)
        if vulkan_dependency.found()
            dav2dplay_deps += vulkan_dependency
            have_vulkan = true
        endif
    endif

    dav2dplay_cflags += '-DHAVE_PLACEBO=' + (have_placebo ? '1' : '0')
    dav2dplay_cflags += '-DHAVE_VULKAN=' + (have_vulkan ? '1' : '0')

    dav2dplay = executable('dav2dplay',
        dav2dplay_sources,
        rev_target,

        link_with : [libdav2d, dav2d_input_objs],
        include_directories : [dav2d_inc_dirs],
        dependencies : [getopt_dependency, dav2dplay_deps],
        install : true,
        c_args : dav2dplay_cflags,
    )
endif
