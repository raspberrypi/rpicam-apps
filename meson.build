project('rpicam-apps', 'c', 'cpp',
        meson_version : '>= 0.64.0',
        version : '1.5.0',
        default_options : [
            'werror=true',
            'warning_level=3',
            'cpp_std=c++17',
            'c_std=c11',
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

dl_dep = dependency('dl', required : true)
libcamera_dep = dependency('libcamera', required : true)

summary({
            'location' : libcamera_dep.get_variable('libdir'),
            'version' : libcamera_dep.version()
        },
        section : 'libcamera')

rpicam_app_src = []
rpicam_app_dep = [libcamera_dep, dl_dep]

subdir('core')
subdir('encoder')
subdir('image')
subdir('output')
subdir('preview')
subdir('utils')

add_project_arguments(cpp_arguments, language : 'cpp')

# Must be put after add_project_arguments as it defines shared library targets.
subdir('post_processing_stages')

rpicam_app = library(
    'rpicam_app',
    rpicam_app_src,
    soversion : meson.project_version(),
    include_directories : include_directories('.'),
    install : true,
    name_prefix : '',
    dependencies : rpicam_app_dep,
)

# Install a symlink to the old library name for legacy purposes.
install_symlink('libcamera_app.so',
                install_dir: get_option('libdir'),
                pointing_to: 'rpicam_app.so')

subdir('apps')

summary({
            'libav encoder' : enable_libav,
            'drm preview' : enable_drm,
            'egl preview' : enable_egl,
            'qt preview' : enable_qt,
            'OpenCV postprocessing' : enable_opencv,
            'TFLite postprocessing' : enable_tflite,
            'Hailo postprocessing' : enable_hailo,
        },
        bool_yn : true, section : 'Build configuration')
