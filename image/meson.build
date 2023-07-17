libcamera_app_src += files([
    'bmp.cpp',
    'dng.cpp',
    'jpeg.cpp',
    'png.cpp',
    'yuv.cpp',
])

image_headers = files([
    'image.hpp',
])

exif_dep = dependency('libexif', required : true)
jpeg_dep = dependency('libjpeg', required : true)
tiff_dep = dependency('libtiff-4', required : true)
png_dep = dependency('libpng', required : true)

libcamera_app_dep += [exif_dep, jpeg_dep, tiff_dep, png_dep]

install_headers(image_headers, subdir: meson.project_name() / 'image')
