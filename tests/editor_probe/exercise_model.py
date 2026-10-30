"""Build the L05 table using genuine palette/inspector widgets and save dialog.

Run inside the bounded editor QA session. This never writes SDF directly: the
Gazebo Model Editor owns all design updates and generates the exported files.
"""
import argparse
import json
from pathlib import Path
import time
from control import Bridge


class ModelExercise:
    def __init__(self, folder):
        self.bridge = Bridge(folder)
        self.folder = Path(folder)
        self.steps = []

    def rows(self):
        return self.bridge.snapshot()

    @staticmethod
    def unique(rows, **conditions):
        matches = [r for r in rows if all(r.get(k) == v for k, v in conditions.items())]
        if len(matches) != 1:
            raise AssertionError('Expected one observed widget {}: {}'.format(conditions, matches))
        return matches[0]

    def tab(self, name, index):
        row = self.unique(self.rows(), name=name, visible=True)
        self.bridge.request('tab', id=row['id'], index=index)

    def initialize(self):
        rows = self.rows()
        action = self.unique(rows, **{'class': 'QAction', 'text': '&Model Editor'})
        if not action.get('checked'):
            self.bridge.request('trigger', id=action['id'])
        self.tab('modelEditorTab', 1)
        rows = self.rows()
        label = self.unique(rows, **{'class': 'QLabel', 'text': 'Static:', 'visible': True})
        checks = [r for r in rows if r['class'] == 'QCheckBox' and r.get('parent') == label['parent']]
        checkbox = min(checks, key=lambda r: abs(r['geometry'][1] - label['geometry'][1]))
        if not checkbox.get('checked'):
            self.bridge.request('click', id=checkbox['id'])
        name = self.unique(rows, **{'class': 'QLineEdit', 'parent': label['parent']})
        self.bridge.request('text', id=name['id'], text='student_table')

    def set_pose(self, xyz):
        rows = self.rows()
        by = {r['id']: r for r in rows}
        groups = []
        for parent in rows:
            labels = {r.get('text') for r in rows if r.get('parent') == parent['id'] and r['class'] == 'QLabel'}
            if not {'X', 'Y', 'Z', 'Roll', 'Pitch', 'Yaw'} <= labels:
                continue
            current, nested = parent, False
            for _ in range(20):
                if current.get('parent') not in by:
                    break
                current = by[current['parent']]
                if current['class'] == 'gazebo::gui::ConfigWidget':
                    break
                if current['class'] == 'gazebo::gui::ConfigChildWidget':
                    nested = True
            spins = [r for r in rows if r.get('parent') == parent['id'] and r['class'] == 'QDoubleSpinBox' and r.get('visible')]
            if not nested and len(spins) == 6:
                groups.append(spins)
        if len(groups) != 1:
            raise AssertionError('Could not uniquely identify link pose controls')
        spins = sorted(groups[0], key=lambda r: (r['geometry'][0], r['geometry'][1]))
        for row, value in zip(spins, list(xyz) + [0, 0, 0]):
            self.bridge.request('value', id=row['id'], value=value)

    def dimensions(self, size):
        rows = self.rows()
        by = {r['id']: r for r in rows}
        groups = []
        for parent in rows:
            labels = {r.get('text') for r in rows if r.get('parent') == parent['id'] and r['class'] == 'QLabel'}
            spins = [r for r in rows if r.get('parent') == parent['id'] and r['class'] == 'QDoubleSpinBox' and r.get('visible')]
            current, geometry = parent, False
            for _ in range(5):
                if current['class'] == 'gazebo::gui::GeometryConfigWidget':
                    geometry = True
                    break
                if current.get('parent') not in by:
                    break
                current = by[current['parent']]
            if geometry and {'X', 'Y', 'Z'} <= labels and len(spins) == 3:
                groups.append(spins)
        if len(groups) != 1:
            raise AssertionError('Could not uniquely identify box size controls: ' + repr(groups))
        for row, value in zip(sorted(groups[0], key=lambda r: r['geometry'][0]), size):
            self.bridge.request('value', id=row['id'], value=value)

    def configure(self, xyz, size):
        self.tab('linkInspectorTab', 0)
        self.set_pose(xyz)
        for index, shape in ((1, 'visual'), (2, 'collision')):
            self.tab('linkInspectorTab', index)
            radio = self.unique(self.rows(), **{'class': 'QRadioButton', 'text': shape, 'visible': True})
            if not radio.get('checked'):
                self.bridge.request('click', id=radio['id'])
            self.dimensions(size)
        button = self.unique(self.rows(), **{'class': 'QPushButton', 'text': 'OK', 'visible': True})
        self.bridge.request('click', id=button['id'])
        self.steps.append({'link_position': xyz, 'visual_collision_size': size})
        (self.folder / 'model-exercise.json').write_text(json.dumps(self.steps, indent=2))

    def add_box(self):
        self.tab('modelEditorTab', 0)
        rows = self.rows()
        box = self.unique(rows, name='modelEditorPaletteBoxButton', visible=True)
        gl = self.unique(rows, name='GLWidget', visible=True)
        x, y = int(gl['rect'][2] * .5), int(gl['rect'][3] * .6)
        self.bridge.request('click', id=box['id'])
        self.bridge.request('mouse_move', id=gl['id'], x=x, y=y)
        self.bridge.request('mouse_click', id=gl['id'], x=x, y=y)
        self.tab('modelEditorTab', 1)
        tree = self.unique(self.rows(), name='modelEditorTreeWidget', visible=True)
        links = [x for x in tree['tree_items'] if len(x['path']) == 2 and x['path'][0] == 2]
        self.bridge.request('tree_double', id=tree['id'], path=links[-1]['path'])


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--io', required=True)
    parser.add_argument('--resume-first', action='store_true')
    args = parser.parse_args()
    driver = ModelExercise(args.io)
    if not args.resume_first:
        driver.initialize()
    parts = [([0, 0, .8], [1.2, .8, .1])]
    parts += [([x, y, .375], [.1, .1, .75]) for x in (-.5, .5) for y in (-.3, .3)]
    for index, (xyz, size) in enumerate(parts):
        if index or not args.resume_first:
            driver.add_box()
        driver.configure(xyz, size)
        print('EDITOR_PART_DONE', index, xyz, size, flush=True)
    rows = driver.rows()
    main_window = driver.unique(rows, name='mainWindow', visible=True)
    driver.bridge.request('screenshot', id=main_window['id'], path=str(driver.folder.resolve() / 'table-editor.png'))
    target = driver.bridge.save_as('student_table')
    driver.bridge.exit_editor('Model')
    print('NATIVE_TABLE_SAVED', target, flush=True)


if __name__ == '__main__':
    main()
