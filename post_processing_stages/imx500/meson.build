imx500_postprocessing_src = files([
    # Base stage
    'imx500_post_processing_stage.cpp',
    # Object detection
    'imx500_object_detection.cpp',
    # Posenet
    'imx500_posenet.cpp',
])

postproc_assets += files([
    assets_dir / 'imx500_mobilenet_ssd.json',
    assets_dir / 'imx500_posenet.json',
])

imx500_postprocessing_lib = shared_module('imx500-postproc', imx500_postprocessing_src,
                                          dependencies : libcamera_dep,
                                          include_directories : '../..',
                                          install : true,
                                          install_dir : posproc_libdir,
                                          name_prefix : '',
                                         )

if get_option('download_imx500_models')
    download_script = meson.project_source_root() / 'utils' / 'download-imx500-models.sh'
    custom_target('imx500-models',
                  command : [ download_script, '@OUTPUT@' ],
                  output : 'imx500-models',
                  install : true,
                  install_dir : get_option('datadir'),
                 )
endif
