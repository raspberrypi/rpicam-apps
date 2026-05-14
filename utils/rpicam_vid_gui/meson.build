# meson.build for rpicam-vid-gui (optional Qt runtime control panel)

enable_control_gui = get_option('enable_control_gui')

qt_dep = dependency('qt6', modules : ['Core', 'Widgets', 'Network'], required : false)

if qt_dep.found()
    qt = import('qt6')
else
    message('Qt6 libraries not found, falling back to Qt5')
    qt_dep = dependency('qt5',
        modules : ['Core', 'Widgets', 'Network'],
        required : enable_control_gui)
    qt = import('qt5')
endif

if qt_dep.found()
    moc_files = qt.preprocess(
        moc_sources : ['main.cpp'],
        dependencies : qt_dep,
        include_directories : include_directories('.'),
    )

    executable('rpicam-vid-gui',
        sources : ['main.cpp', moc_files],
        dependencies : qt_dep,
        include_directories : include_directories('../..'),
        install : true,
    )
endif
