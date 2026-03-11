Windows shell basics: `dir`, `Get-ChildItem`, `cd`, `rg --files`, `rg <pattern> <path>`, `type <file>`, `git status`.
Build/flash: open `MDK-ARM/Dart_Ctrl.uvprojx` in Keil uVision; build/flash from Keil (no CLI build documented).
Entry point: `Core/Src/main.c` (calls Module_Init in `User/src/UserTask.c`).