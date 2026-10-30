# Real Classic editor QA

This test-only GUIPlugin uses the existing Gazebo/Qt SDK and is not loaded by
normal lessons. No private ModelCreator or BuildingMaker headers are required.
It exposes widgets, actions and graphics-view bounds from its own client to a
local JSON command/response directory. QObject targets must come from a live
snapshot. Native X11 double clicks verify the tracked client PID/session token
and translate an observed viewport point; other application windows are excluded.

Flow: create a private lesson workspace → preload its model parent into a
private GUI INI → launch an owned gzclient with this plugin → use actual palette
buttons, inspector fields and Save As → inspect the generated packages → reload
them in physical acceptance → stop only the owned session.

File actions and shortcuts acknowledge before entering a modal dialog. A queued
Qt callback opens the dialog after the poll handler returns, allowing the poll
timer to keep handling its genuine name/location/Save controls. Screenshots use
the owned native window so OpenGL is included.

```sh
cmake -S tests/editor_probe -B .deploy/editor-probe-build
cmake --build .deploy/editor-probe-build -j4
python3 tests/editor_probe/session.py --io .deploy/editor-qa-NEW --lesson 05
```

In another terminal in the same repository, using that exact new directory:

```sh
python3 tests/editor_probe/exercise_model.py --io .deploy/editor-qa-NEW
python3 tests/editor_probe/exercise_building.py --io .deploy/editor-qa-NEW
touch .deploy/editor-qa-NEW/stop
```

The session has a finite maximum lifetime and always cleans its owned processes.
Its session.json records the original home GUI INI hash and verifies it remains
unchanged. Preloading the save parent is necessary because Classic Save As has a
path that ignores the GUI INI override. No HOME override or global SSH/system
change is used. Never run overlapping GUI checks that independently change the
user GUI INI while checking this invariant.

The current 2-D exercise expects the native editor's default 100 px/m view. The
initial rectangle is approximate; actual Wall Inspector endpoints and thickness
refine it, and Door Inspector sets the opening. Every output path is in the
private workspace. Existing exported packages are rejected instead of replaced.
These QA scripts do not mark a learner's GUI skills as complete.
