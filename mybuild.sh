meson setup build -Denable_libav=true -Denable_drm=true -Denable_egl=true -Denable_qt=true -Denable_opencv=true -Denable_tflite=false
meson compile -C build
sudo meson install -C build
