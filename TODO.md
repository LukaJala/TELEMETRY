# LVGL Sim CI Fix TODO
- [x] Create TODO.md
- [x] Update .github/workflows/lvgl-sim.yml with fixes for black screen (better patching, use lvgl_demo_ui.c, stable runtime)
- [x] Fix lv_conf.h error, revert to original patching method
- [x] Switch back to ui_init with better sed patterns, pre-sanitize ui.c before copy
- [x] Disable interfering slideshow call in main.c to prevent UI overwrite
