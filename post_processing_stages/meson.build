posproc_libdir = get_option('prefix') / get_option('libdir') / 'rpicam-apps-postproc'

conf_data = configuration_data()
conf_data.set('POSTPROC_LIB_DIR', '"' + posproc_libdir + '"')
configure_file(output : 'postproc_lib.h', configuration : conf_data)

# Core postprocessing framework files.
rpicam_app_src += files([
    'histogram.cpp',
    'post_processing_stage.cpp',
    'pwl.cpp',
])

# Core postprocessing stages.
core_postproc_src = files([
    'hdr_stage.cpp',
    'motion_detect_stage.cpp',
    'negate_stage.cpp',
])

core_postproc_lib = shared_module('core-postproc', core_postproc_src,
                                  include_directories : '../..',
                                  dependencies : libcamera_dep,
                                  cpp_args : cpp_arguments,
                                  install : true,
                                  install_dir : posproc_libdir,
                                  name_prefix : '',
                                 )

# OpenCV based postprocessing stages.
enable_opencv = false
opencv_dep = dependency('opencv4', required : get_option('enable_opencv'))
if opencv_dep.found()
    opencv_postproc_src = files([
        'sobel_cv_stage.cpp',
        'face_detect_cv_stage.cpp',
        'annotate_cv_stage.cpp',
        'plot_pose_cv_stage.cpp',
        'object_detect_draw_cv_stage.cpp',
    ])

    opencv_postproc_lib = shared_module('opencv-postproc', opencv_postproc_src,
                                        include_directories : '../..',
                                        dependencies : [libcamera_dep, opencv_dep],
                                        cpp_args : cpp_arguments,
                                        install : true,
                                        install_dir : posproc_libdir,
                                        name_prefix : '',
                                       )
    enable_opencv = true
endif

# TFlite based postprocessing stages.
enable_tflite = false
tflite_dep = dependency('tensorflow-lite', required : get_option('enable_tflite'))
if tflite_dep.found()
    tflite_postproc_src = files([
        'tf_stage.cpp',
        'object_classify_tf_stage.cpp',
        'pose_estimation_tf_stage.cpp',
        'object_detect_tf_stage.cpp',
        'segmentation_tf_stage.cpp',
    ])

    tflite_postproc_lib = shared_module('tflite-postproc', tflite_postproc_src,
                                        include_directories : '../..',
                                        dependencies : [libcamera_dep, tflite_dep],
                                        cpp_args : cpp_arguments,
                                        install : true,
                                        install_dir : posproc_libdir,
                                        name_prefix : '',
                                       )
    enable_tflite = true
endif

# Hailo postprocessing stages.
enable_hailo = false
hailort_dep = dependency('HailoRT', modules : ['HailoRT::libhailort'], version: '>=4.17.0',
                         required : get_option('enable_hailo'))
hailo_tappas_dep = dependency('hailo-tappas-core', required : get_option('enable_hailo'))
if hailort_dep.found() and hailo_tappas_dep.found()
    subdir('hailo')
    enable_hailo = true
endif

post_processing_headers = files([
    'histogram.hpp',
    'object_detect.hpp',
    'post_processing_stage.hpp',
    'pwl.hpp',
    'segmentation.hpp',
    'tf_stage.hpp',
])

install_headers(post_processing_headers, subdir: meson.project_name() / 'post_processing_stages')
