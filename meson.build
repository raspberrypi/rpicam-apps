project('rpicam-apps', 'c', 'cpp',
        meson_version : '>= 0.64.0',
        version : '1.5.2',
        default_options : [
            'werror=false',
            # 'warning_level=3',
            'cpp_std=c++17',
            'c_std=c11',
            'buildtype=release',
        ],
        license : 'BSD-2-Clause')

meson.add_dist_script('utils' / 'gen-dist.sh')

fs = import('fs')

# cpp_arguments = ['-pedantic', '-Wno-unused-parameter', '-faligned-new']
cpp_arguments = ['-w', '-faligned-new']

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
cmake = import('cmake')

# Configure the CMake project
sub_proj = cmake.subproject('kakadujs')

# Fetch the dependency object
kakadu_lib = sub_proj.dependency('kakadu')
kakaduapp_lib = sub_proj.dependency('kakaduappsupport')
# Must be put after add_project_arguments as it defines shared library targets.
subdir('post_processing_stages')

# Generate a version string.
version_cmd = [meson.project_source_root() / 'utils' / 'version.py', meson.project_version()]

# Check if a version.gen file is present.
# This would have been generated from the meson dist command.
dist_version_file = meson.project_source_root() / 'version.gen'
if fs.is_file(dist_version_file)
    version_cmd += fs.read(dist_version_file)
endif

version_cpp = vcs_tag(command : version_cmd,
                      replace_string: '@VER@',
                      input : meson.project_source_root() / 'core' / 'version.cpp.in',
                      output : 'version.cpp',
                      fallback : meson.project_version())

rpicam_app_src += version_cpp

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
            'IMX500 postprocessing' : get_option('enable_imx500'),
        },
        bool_yn : true, section : 'Build configuration')
