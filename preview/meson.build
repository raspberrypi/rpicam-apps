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
    rpicam_app_dep += drm_deps
    rpicam_app_src += files('drm_preview.cpp')
    cpp_arguments += '-DLIBDRM_PRESENT=1'
    enable_drm = true
endif

enable_egl = false
x11_deps = dependency('x11', required : get_option('enable_egl'))
epoxy_deps = dependency('epoxy', required : get_option('enable_egl'))

if x11_deps.found() and epoxy_deps.found()
    rpicam_app_dep += [x11_deps, epoxy_deps]
    rpicam_app_src += files('egl_preview.cpp')
    cpp_arguments += '-DLIBEGL_PRESENT=1'
    enable_egl = true
endif

enable_qt = false
qt_dep = dependency('qt5', modules : ['Core', 'Widgets'], required : get_option('enable_qt'))
if qt_dep.found()
    rpicam_app_dep += qt_dep
    rpicam_app_src += files('qt_preview.cpp')
    cpp_arguments += '-DQT_PRESENT=1'
    enable_qt = true
endif

install_headers(preview_headers, subdir: meson.project_name() / 'preview')
