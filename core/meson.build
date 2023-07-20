boost_dep = dependency('boost', modules : ['program_options'], required : true)
thread_dep = dependency('threads', required : true)

libcamera_app_dep += [boost_dep, thread_dep]

libcamera_app_src += files([
    'buffer_sync.cpp',
    'dma_heaps.cpp',
    'libcamera_app.cpp',
    'options.cpp',
    'post_processor.cpp',
])

core_headers = files([
    'buffer_sync.hpp',
    'completed_request.hpp',
    'dma_heaps.hpp',
    'frame_info.hpp',
    'libcamera_app.hpp',
    'libcamera_encoder.hpp',
    'logging.hpp',
    'metadata.hpp',
    'options.hpp',
    'post_processor.hpp',
    'still_options.hpp',
    'stream_info.hpp',
    'version.hpp',
    'video_options.hpp',
])

# Generate a version string.
version_cmd = [meson.project_source_root() / 'utils' / 'version.py']

# Check if a version.gen file is present.
# This would have been generated from the meson dist command.
fs = import('fs')
dist_version_file = meson.project_source_root() / 'version.gen'
if fs.is_file(dist_version_file)
    version_cmd += fs.read(dist_version_file)
endif

version_cpp = vcs_tag(command : version_cmd,
                      replace_string: '@VER@',
                      input : 'version.cpp.in',
                      output : 'version.cpp',
                      fallback : meson.project_version())

libcamera_app_src += version_cpp

install_headers(core_headers, subdir: meson.project_name() / 'core')
