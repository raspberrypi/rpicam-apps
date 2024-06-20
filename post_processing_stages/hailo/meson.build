hailo_tappas_lib_dir = hailo_tappas_dep.get_variable('tappas_libdir')
hailo_tappas_posproc_libdir = hailo_tappas_lib_dir / 'post-process'

hailo_conf_data = configuration_data()
hailo_conf_data.set('HAILO_POSTPROC_LIB_DIR', '"' + hailo_tappas_posproc_libdir + '"')
hailo_postproc_lib = configure_file(output : 'hailo_postproc_lib.h', configuration : hailo_conf_data)

hailo_deps = [hailort_dep, hailo_tappas_dep, libcamera_dep]

# Hailo Tappas PP config files to be installed
hailopp_config_files = files([
    assets_dir / 'yolov5_personface.json',
])

hailo_postprocessing_src = files([
    # Base stage
    'hailo_postprocessing_stage.cpp',
    # Yolo 5/6/8/x inference
    'hailo_yolo_inference.cpp',
    # Image classifier
    'hailo_classifier.cpp',
])

postproc_assets += files([
    assets_dir / 'hailo_classifier.json',
    assets_dir / 'hailo_yolov5_personface.json',
    assets_dir / 'hailo_yolov6_inference.json',
    assets_dir / 'hailo_yolov8_inference.json',
    assets_dir / 'hailo_yolox_inference.json',
])

if opencv_dep.found()
    hailo_postprocessing_src += files([
        # Pose estimation
        'hailo_yolov8_pose.cpp',
        # Instance segmentation
        'hailo_yolov5_segmentation.cpp'
    ])

    postproc_assets += files([
        assets_dir / 'hailo_yolov8_pose.json',
        assets_dir / 'hailo_yolov5_segmentation.json',
    ])

    hailo_deps + opencv_dep
    hailopp_config_files += files([
        assets_dir / 'yolov5seg.json',
    ])
endif

hailo_cpp_arguments = ['-Wno-ignored-qualifiers', '-Wno-unused-parameter', '-Wno-extra']

hailo_postprocessing_lib = shared_module('hailo-postproc', hailo_postprocessing_src,
                                         dependencies : hailo_deps,
                                         cpp_args : hailo_cpp_arguments,
                                         include_directories : '../..',
                                         install : true,
                                         install_dir : posproc_libdir,
                                         name_prefix : '',
                                        )

install_data(hailopp_config_files,
             install_dir : get_option('datadir') / 'hailo-models')

if get_option('download_hailo_models')
    download_script = meson.project_source_root() / 'utils' / 'download-hailo-models.sh'
    custom_target('hailo-models',
                  command : [ download_script, '@OUTPUT@' ],
                  output : 'hailo-models',
                  install : true,
                  install_dir : get_option('datadir'),
                 )
endif
