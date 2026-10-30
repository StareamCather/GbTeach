# Native Classic 11 editor export regression fixtures

These two packages were created and saved in the actual ikaros Gazebo 11.15.1
Model Editor and Building Editor on 2026-10-30. They are test artifacts; they
are not copied into a learner's workspace as completed student work.

The table was built with five palette boxes, edited through the Link/Visual/
Collision inspectors, marked Static, and saved with the native Save As dialog.
Its generated model origin is 0.46 m above the feet. The room was drawn with
four Wall tools, refined with Wall Inspector, given a 0.8 x 1.4 m Door opening,
and saved with Building Editor. Its front wall becomes three collision boxes.
Native SDF serialization uses angles such as 1.5708 and 3.14159.

Run real reloading and physical checks on new private copies:

```sh
python3 tests/lesson05_acceptance.py \
  --export-table tests/fixtures/editor_exports/student_table \
  --export-room tests/fixtures/editor_exports/student_room \
  --report .deploy/lesson05-native-scenarios.json
```

The source files remain unchanged. Passing these fixture checks proves saved
resources can reload; it does not mark a learner's mouse workflow complete.
