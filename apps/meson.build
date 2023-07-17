libcamera_still = executable('libcamera-still', files('libcamera_still.cpp'),
                             include_directories : include_directories('..'),
                             dependencies: [libcamera_dep, boost_dep],
                             link_with : libcamera_app,
                             install : true)

libcamera_vid = executable('libcamera-vid', files('libcamera_vid.cpp'),
                           include_directories : include_directories('..'),
                           dependencies: [libcamera_dep, boost_dep],
                           link_with : libcamera_app,
                           install : true)

libcamera_hello = executable('libcamera-hello', files('libcamera_hello.cpp'),
                             include_directories : include_directories('..'),
                             dependencies: libcamera_dep,
                             link_with : libcamera_app,
                             install : true)

libcamera_raw = executable('libcamera-raw', files('libcamera_raw.cpp'),
                           include_directories : include_directories('..'),
                           dependencies: [libcamera_dep, boost_dep],
                           link_with : libcamera_app,
                           install : true)

libcamera_jpeg = executable('libcamera-jpeg', files('libcamera_jpeg.cpp'),
                            include_directories : include_directories('..'),
                            dependencies: [libcamera_dep, boost_dep],
                            link_with : libcamera_app,
                            install : true)

if enable_tflite
    libcamera_detect = executable('libcamera-detect', files('libcamera_detect.cpp'),
                                  include_directories : include_directories('..'),
                                  dependencies: [libcamera_dep, boost_dep],
                                  link_with : libcamera_app,
                                  install : true)
endif
