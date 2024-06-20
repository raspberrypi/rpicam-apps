imx500_postprocessing_src = files([
    # Base stage
    'imx500_post_processing_stage.cpp',
    # Object inference
    'imx500_object_inference.cpp',
    # Posenet
    'imx500_posenet.cpp',
])

postproc_assets += files([
    asset_dir / 'imx500_mobilenet_ssd.json',
    asset_dir / 'imx500_posenet.json',
])

imx500_postprocessing_lib = shared_module('imx500-postproc', imx500_postprocessing_src,
                                          dependencies : libcamera_dep,
                                          include_directories : '../..',
                                          install : true,
                                          install_dir : posproc_libdir,
                                          name_prefix : '',
                                         )
