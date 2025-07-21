preview_libdir = get_option('prefix') / get_option('libdir') / 'rpicam-apps-preview'

conf_data.set('PREVIEW_LIB_DIR', '"' + preview_libdir + '"')

rpicam_app_src += files([
    'null_preview.cpp',
    'preview.cpp',
])

preview_headers = files([
    'preview.hpp',
])

enable_drm = false
drm_deps = dependency('libdrm', required : get_option('enable_drm'))

if drm_deps.found()
    drm_lib = shared_module('drm-preview', files('drm_preview.cpp'),
                            include_directories : '../',
                            dependencies : [drm_deps, libcamera_dep],
                            cpp_args : cpp_arguments,
                            install : true,
                            install_dir: preview_libdir,
                            name_prefix : '',
    )
    conf_data.set('LIBDRM_PRESENT', 1)
    enable_drm = true
endif

enable_egl = false
x11_deps = dependency('x11', required : get_option('enable_egl'))
epoxy_deps = dependency('epoxy', required : get_option('enable_egl'))

if x11_deps.found() and epoxy_deps.found()
    egl_lib = shared_module('egl-preview', files('egl_preview.cpp'),
                            include_directories : '../',
                            dependencies : [x11_deps, epoxy_deps, libcamera_dep],
                            cpp_args : cpp_arguments,
                            install : true,
                            install_dir: preview_libdir,
                            name_prefix : '',
    )
    conf_data.set('LIBEGL_PRESENT', 1)
    enable_egl = true
endif

enable_qt = false
qt_dep = dependency('qt5', modules : ['Core', 'Widgets'], required : get_option('enable_qt'))
if qt_dep.found()
    egl_lib = shared_module('qt-preview', files('qt_preview.cpp'),
                            include_directories : '../',
                            dependencies : [qt_dep, libcamera_dep],
                            cpp_args : cpp_arguments,
                            install : true,
                            install_dir: preview_libdir,
                            name_prefix : '',
    )
    conf_data.set('QT_PRESENT', 1)
    enable_qt = true
endif

install_headers(preview_headers, subdir: meson.project_name() / 'preview')
