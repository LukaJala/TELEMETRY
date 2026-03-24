#!/bin/bash
git clone --recursive https://github.com/lvgl/lv_port_linux.git
cd lv_port_linux
sed -i 's/\\&amp;/\&/g' ../SCREEN_TEST/SCREEN_TEST/main/ui.c
cp ../SCREEN_TEST/SCREEN_TEST/main/ui.* src/ui/
sed -i '/#include "lvgl\\/lvgl.h"/a #include "ui/ui.h"' src/main.c
sed -i 's/lv_demo_widgets()/ui_init(lv_display_get_default())/' src/main.c
sed -i 's/lv_demo_benchmark()/ui_init(lv_display_get_default())/' src/main.c
sed -i 's/lv_demo_music()/ui_init(lv_display_get_default())/' src/main.c
sed -i 's/lv_demo_stress()/ui_init(lv_display_get_default())/' src/main.c
sed -i 's|src/main.c|src/main.c\n    src/ui/ui.c|' CMakeLists.txt
sed -i 's|include_directories(src)|include_directories(src)\ninclude_directories(src/ui)|' CMakeLists.txt
mkdir -p build
cd build
cmake .. -G Ninja -DCMAKE_BUILD_TYPE=Release
ninja
DISPLAY=:99 Xvfb :99 -screen 0 800x600x24 & 
sleep 2
SDL_VIDEODRIVER=x11 ./bin/lvglsim &
sleep 15
scrot screenshot.png
killall lvglsim Xvfb
eog screenshot.png # View screenshot

