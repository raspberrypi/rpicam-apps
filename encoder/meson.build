rpicam_app_src += files([
    'encoder.cpp',
    'h264_encoder.cpp',
    'mjpeg_encoder.cpp',
    'null_encoder.cpp',
])

encoder_headers = files([
    'encoder.hpp',
    'h264_encoder.hpp',
    'mjpeg_encoder.hpp',
    'null_encoder.hpp',
])

libav_dep_names = ['libavcodec', 'libavdevice', 'libavformat', 'libavutil', 'libswresample']
libav_deps = []

enable_libav = true
foreach name : libav_dep_names
    dep = dependency(name, required : get_option('enable_libav'))
    if not dep.found()
        enable_libav = false
        break
    endif
    libav_deps += dep
endforeach

# Add ALSA library dependency for direct ALSA support
alsa_dep = dependency('alsa', required : true)
if alsa_dep.found()
    cpp_arguments += '-DALSA_PRESENT=1'
endif

if enable_libav
        rpicam_app_src += files('libav_encoder.cpp')
        encoder_headers += files('libav_encoder.hpp')
        rpicam_app_dep += libav_deps
        if alsa_dep.found()
            rpicam_app_dep += alsa_dep
        endif
        cpp_arguments += '-DLIBAV_PRESENT=1'
endif

install_headers(encoder_headers, subdir: meson.project_name() / 'encoder')
