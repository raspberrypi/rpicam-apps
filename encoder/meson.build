libcamera_app_src += files([
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

enable_libav = get_option('enable_libav')
libav_dep_names = ['libavcodec', 'libavdevice', 'libavformat', 'libavutil', 'libswresample']
libav_deps = []

if enable_libav
    foreach name : libav_dep_names
        dep = dependency(name, required : false)
        if not dep.found()
            enable_libav = false
            break
        endif
        libav_deps += dep
    endforeach
endif

if enable_libav
        libcamera_app_src += files('libav_encoder.cpp')
        encoder_headers += files('libav_encoder.hpp')
        libcamera_app_dep += libav_deps
        cpp_arguments += '-DLIBAV_PRESENT=1'
endif

install_headers(encoder_headers, subdir: meson.project_name() / 'encoder')
