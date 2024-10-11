boost_dep = dependency('boost', modules : ['program_options'], required : true)
thread_dep = dependency('threads', required : true)

rpicam_app_dep += [boost_dep, thread_dep]

rpicam_app_src += files([
    'buffer_sync.cpp',
    'dma_heaps.cpp',
    'rpicam_app.cpp',
    'options.cpp',
    'post_processor.cpp',
])

core_headers = files([
    'buffer_sync.hpp',
    'completed_request.hpp',
    'dma_heaps.hpp',
    'frame_info.hpp',
    'rpicam_app.hpp',
    'rpicam_encoder.hpp',
    'logging.hpp',
    'metadata.hpp',
    'options.hpp',
    'post_processor.hpp',
    'still_options.hpp',
    'stream_info.hpp',
    'version.hpp',
    'video_options.hpp',
])

install_headers(core_headers, subdir: meson.project_name() / 'core')

# Install symlinks to the old header directories and names for legacy purposes.
install_symlink('libcamera-apps',
                install_dir: get_option('includedir'),
                pointing_to: 'rpicam-apps')

inc_install_dir = get_option('includedir') / meson.project_name() / 'core'
install_symlink('libcamera_app.h',
                install_dir: inc_install_dir,
                pointing_to: 'rpicam_app.hpp')

install_symlink('libcamera_encoder.h',
                install_dir: inc_install_dir,
                pointing_to: 'rpicam_encoder.hpp')
