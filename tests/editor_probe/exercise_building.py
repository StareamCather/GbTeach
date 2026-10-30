"""Exercise actual 2-D Wall/Door tools and their native inspectors."""
import argparse
import json
import time
from pathlib import Path
from control import Bridge
from native_input import double_click


class BuildingExercise:
    def __init__(self, folder):
        self.folder = Path(folder)
        self.bridge = Bridge(folder)
        self.steps = []

    def rows(self):
        return self.bridge.snapshot()

    def one(self, **values):
        matches = [r for r in self.rows() if all(r.get(k) == v for k, v in values.items())]
        if len(matches) != 1:
            raise AssertionError('Expected one observed widget {}: {}'.format(values, matches))
        return matches[0]

    def viewport(self):
        return self.one(name='editorView', visible=True)['viewport']

    def inspect_at(self, x, y):
        target = self.viewport()
        double_click(self.bridge, target, x, y)

    def wall(self, endpoints):
        rows = self.rows()
        dialog = self.one(name='wallInspectorDialog', visible=True)
        spins = [r for r in rows if r['class'] == 'QDoubleSpinBox' and r.get('visible')]
        coordinates = [r for r in spins if r.get('parent') != dialog['id']]
        if len(coordinates) != 4:
            raise AssertionError('Wall inspector must expose start/end XY')
        for row, value in zip(sorted(coordinates, key=lambda r: (r['geometry'][0], r['geometry'][1])), endpoints):
            self.bridge.request('value', id=row['id'], value=value)
        for prefix, value in [('Height', 2), ('Thickness', .12)]:
            label = next(r for r in rows if r['class'] == 'QLabel' and r.get('parent') == dialog['id']
                         and r.get('text', '').strip().startswith(prefix))
            spin = min([r for r in spins if r.get('parent') == dialog['id']],
                       key=lambda r: abs(r['geometry'][1] - label['geometry'][1]))
            self.bridge.request('value', id=spin['id'], value=value)
        button = next(r for r in self.rows() if r.get('visible') and r['class'] == 'QPushButton'
                      and r.get('parent') == dialog['id'] and r.get('text', '').replace('&', '').upper() == 'OK')
        self.bridge.request('click', id=button['id'])
        self.steps.append({'wall_start_end_m': endpoints, 'height_m': 2, 'thickness_m': .12})
        (self.folder / 'building-exercise.json').write_text(json.dumps(self.steps, indent=2))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--io', required=True)
    parser.add_argument('--resume-first-wall', action='store_true')
    args = parser.parse_args()
    driver = BuildingExercise(args.io)
    if not args.resume_first_wall:
        action = driver.one(**{'class': 'QAction', 'text': '&Building Editor'})
        if not action.get('checked'):
            driver.bridge.request('trigger', id=action['id'])
        rows = driver.rows()
        name = next(r for r in rows if r['class'] == 'QLineEdit' and r.get('visible'))
        driver.bridge.request('text', id=name['id'], text='student_room')
    rows = driver.rows()
    view = driver.one(name='editorView', visible=True)
    viewport = next(r for r in rows if r['id'] == view['viewport'])
    cx, cy = viewport['rect'][2] / 2, viewport['rect'][3] / 2
    # In the current native EditorView the grid scale is 100px/m. Initial
    # approximate clicks are refined through inspector endpoints in meters.
    if not args.resume_first_wall:
        wall = driver.one(**{'class': 'QToolButton', 'text': 'Wall', 'visible': True})
        driver.bridge.request('click', id=wall['id'])
        for x, y in [(cx-150, cy-125), (cx+150, cy-125), (cx+150, cy+125), (cx-150, cy+125), (cx-150, cy-125)]:
            driver.bridge.request('mouse_move', id=view['viewport'], x=round(x), y=round(y))
            driver.bridge.request('mouse_click', id=view['viewport'], x=round(x), y=round(y))
        driver.bridge.request('key', id=view['viewport'], key=16777216)
        driver.inspect_at(cx, cy-125)
    top, bottom = -cy / 100 + 1.19, -cy / 100 - 1.19
    driver.wall([-1.44, top, 1.44, top])
    for point, ends in [((cx+144, cy), [1.44, top, 1.44, bottom]),
                        ((cx, cy+119), [1.44, bottom, -1.44, bottom]),
                        ((cx-144, cy), [-1.44, bottom, -1.44, top])]:
        driver.inspect_at(*point)
        driver.wall(ends)
    door = driver.one(**{'class': 'QToolButton', 'text': 'Door', 'visible': True})
    driver.bridge.request('click', id=door['id'])
    driver.bridge.request('mouse_move', id=view['viewport'], x=round(cx), y=round(cy+119))
    driver.bridge.request('mouse_click', id=view['viewport'], x=round(cx), y=round(cy+119))
    driver.inspect_at(cx, cy+119)
    rows = driver.rows()
    dialog = driver.one(name='windowDoorInspectorDialog', visible=True)
    for prefix, value in [('Width', .8), ('Height', 1.4), ('Elevation', 0)]:
        label = next(r for r in rows if r.get('visible') and r['class'] == 'QLabel' and r.get('text', '').startswith(prefix))
        spins = [r for r in rows if r['class'] == 'QDoubleSpinBox' and r.get('parent') == label['parent']]
        spin = min(spins, key=lambda r: abs(r['geometry'][1] - label['geometry'][1]))
        driver.bridge.request('value', id=spin['id'], value=value)
    ok = next(r for r in driver.rows() if r.get('visible') and r['class'] == 'QPushButton'
              and r.get('parent') == dialog['id'] and r.get('text', '').replace('&', '') == 'OK')
    driver.bridge.request('click', id=ok['id'])
    main_window = driver.one(name='mainWindow', visible=True)
    driver.bridge.request('screenshot', id=main_window['id'], path=str(driver.folder.resolve() / 'building-editor.png'))
    target = driver.bridge.save_as('student_room')
    driver.bridge.exit_editor('Building')
    print('NATIVE_ROOM_SAVED', target, flush=True)


if __name__ == '__main__':
    main()
