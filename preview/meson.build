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

enable_wayland = false
wl_client_dep = dependency('wayland-client', required : get_option('enable_wayland'))
wl_egl_dep = dependency('wayland-egl', required : get_option('enable_wayland'))
# The Wayland preview needs epoxy too, but it must be looked up here rather than
# reusing the egl section's result, so that it works independently of enable_egl.
wl_epoxy_dep = dependency('epoxy', required : get_option('enable_wayland'))
# wayland-protocols ships the xdg-shell / xdg-decoration XML descriptions that
# wayland-scanner turns into client glue; without it we cannot build the preview.
wl_protocols_dep = dependency('wayland-protocols', required : get_option('enable_wayland'))
wl_scanner = find_program('wayland-scanner', required : get_option('enable_wayland'))

if wl_client_dep.found() and wl_egl_dep.found() and wl_epoxy_dep.found() and wl_protocols_dep.found() and wl_scanner.found()
    wl_protocol_dir = wl_protocols_dep.get_variable(pkgconfig : 'pkgdatadir')
    xdg_shell_xml = wl_protocol_dir / 'stable' / 'xdg-shell' / 'xdg-shell.xml'
    xdg_decoration_xml = wl_protocol_dir / 'unstable' / 'xdg-decoration' / 'xdg-decoration-unstable-v1.xml'

    xdg_shell_header = custom_target('xdg-shell-client-protocol.h',
                                     input : xdg_shell_xml,
                                     output : 'xdg-shell-client-protocol.h',
                                     command : [wl_scanner, 'client-header', '@INPUT@', '@OUTPUT@'],
    )
    xdg_shell_code = custom_target('xdg-shell-protocol.c',
                                   input : xdg_shell_xml,
                                   output : 'xdg-shell-protocol.c',
                                   command : [wl_scanner, 'private-code', '@INPUT@', '@OUTPUT@'],
    )
    xdg_decoration_header = custom_target('xdg-decoration-unstable-v1-client-protocol.h',
                                          input : xdg_decoration_xml,
                                          output : 'xdg-decoration-unstable-v1-client-protocol.h',
                                          command : [wl_scanner, 'client-header', '@INPUT@', '@OUTPUT@'],
    )
    xdg_decoration_code = custom_target('xdg-decoration-unstable-v1-protocol.c',
                                        input : xdg_decoration_xml,
                                        output : 'xdg-decoration-unstable-v1-protocol.c',
                                        command : [wl_scanner, 'private-code', '@INPUT@', '@OUTPUT@'],
    )

    # The generated protocol C is built separately with warnings relaxed, as it
    # does not satisfy the project's -Werror/-Wpedantic settings.
    xdg_shell_lib = static_library('xdg-shell-protocol', [xdg_shell_code, xdg_decoration_code],
                                   dependencies : wl_client_dep,
                                   c_args : ['-w'],
                                   override_options : ['werror=false', 'warning_level=0'],
    )

    wayland_lib = shared_module('wayland-egl-preview',
                                [files('wayland_egl_preview.cpp'), xdg_shell_header, xdg_decoration_header],
                                include_directories : '../',
                                dependencies : [wl_client_dep, wl_egl_dep, wl_epoxy_dep, libcamera_dep],
                                link_with : xdg_shell_lib,
                                cpp_args : cpp_arguments,
                                install : true,
                                install_dir: preview_libdir,
                                name_prefix : '',
    )
    conf_data.set('WAYLAND_PRESENT', 1)
    enable_wayland = true
endif

enable_qt = false

if not get_option('enable_qt').disabled()
    qt_dep = dependency('qt6', modules : ['Core', 'Widgets'], required : false)

    if not qt_dep.found()
        message('Qt6 libraries not found, falling back to Qt5')
        qt_dep = dependency('qt5', modules : ['Core', 'Widgets'], required : get_option('enable_qt'))
    endif

    if qt_dep.found()
        qt_lib = shared_module('qt-preview', files('qt_preview.cpp'),
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
endif

install_headers(preview_headers, subdir: meson.project_name() / 'preview')
