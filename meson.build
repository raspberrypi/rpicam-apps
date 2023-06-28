project('libcamera-apps', 'c', 'cpp',
        meson_version : '>= 0.64.0',
        version : '1.2.0',
        default_options : [
            'werror=true',
            'warning_level=3',
            'cpp_std=c++17',
            'buildtype=release',
        ],
        license : 'BSD-2-Clause')

meson.add_dist_script('utils' / 'gen-dist.sh')

cpp_arguments = ['-pedantic', '-Wno-unused-parameter', '-faligned-new']

# Needed for file sizes > 32-bits.
cpp_arguments += '-D_FILE_OFFSET_BITS=64'

cxx = meson.get_compiler('cpp')
cpu = host_machine.cpu()
neon = get_option('neon_flags')

if cxx.get_id() == 'gcc'
    cpp_arguments += '-Wno-psabi'
endif

if cpu == 'aarch64' or neon == 'arm64'
    cpp_arguments += '-ftree-vectorize'
elif neon == 'armv8-neon'
    cpp_arguments += ['-mfpu=neon-fp-armv8', '-ftree-vectorize']
endif

libcamera_dep = dependency('libcamera', required : true)

summary({
            'location' : libcamera_dep.get_variable('libdir'),
            'version' : libcamera_dep.version()
        },
        section : 'libcamera')

libcamera_app_src = []
libcamera_app_dep = [libcamera_dep]

subdir('core')
subdir('encoder')
subdir('image')
subdir('output')
subdir('preview')
subdir('post_processing_stages')
subdir('utils')

add_project_arguments(cpp_arguments, language : 'cpp')

libcamera_app = library(
    'libcamera_app',
    libcamera_app_src,
    soversion : meson.project_version(),
    include_directories : include_directories('.'),
    install : true,
    name_prefix : '',
    dependencies : libcamera_app_dep,
)

subdir('apps')

summary({
            'libav encoder' : enable_libav,
            'drm preview' : enable_drm,
            'egl preview' : enable_egl,
            'qt preview' : enable_qt,
            'OpenCV postprocessing' : enable_opencv,
            'TFLite postprocessing' : enable_tflite,
        },
        bool_yn : true, section : 'Build configuration')
