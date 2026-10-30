"""Control only the test plugin in one owned Gazebo client."""
import json
import os
import time
from pathlib import Path


class Bridge:
    def __init__(self, folder):
        self.folder = Path(folder)

    def request(self, verb, timeout=8, **values):
        sequence = 1
        for name in ('command.json', 'response.json'):
            path = self.folder / name
            if path.exists():
                sequence = max(sequence, json.loads(path.read_text()).get('seq', 0) + 1)
        command = dict(values, verb=verb, seq=sequence)
        temporary = self.folder / 'command.tmp'
        temporary.write_text(json.dumps(command), encoding='utf-8')
        os.replace(str(temporary), str(self.folder / 'command.json'))
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            response = self.folder / 'response.json'
            if response.exists():
                data = json.loads(response.read_text())
                if data.get('seq') == sequence:
                    if not data.get('ok'):
                        raise RuntimeError(data.get('error'))
                    return data.get('data', {})
            time.sleep(.05)
        raise TimeoutError('Owned editor plugin did not answer ' + verb)

    def snapshot(self):
        data = self.request('snapshot')['objects']
        (self.folder / 'snapshot.json').write_text(json.dumps(data, ensure_ascii=False, indent=2))
        return data

    def save_as(self, name):
        rows = self.snapshot()
        main = next(r for r in rows if r.get('name') == 'mainWindow')
        self.request('key', id=main['id'], key=ord('S'), modifiers=0x06000000)
        deadline = time.monotonic() + 6
        while time.monotonic() < deadline:
            rows = self.snapshot()
            dialogs = [r for r in rows if r.get('name') == 'saveDialog' and r.get('visible')]
            if dialogs:
                break
        else:
            raise AssertionError('Native Save As dialog did not open')
        dialog = dialogs[0]
        lines = sorted([r for r in rows if r['class'] == 'QLineEdit'
                        and r.get('parent') == dialog['id'] and r.get('visible')],
                       key=lambda r: r['geometry'][1])
        if len(lines) != 2:
            raise AssertionError('Expected native model name and location fields')
        workspace = Path(json.loads((self.folder / 'session.json').read_text())['workspace'])
        target = workspace / 'models' / name
        if target.exists():
            raise FileExistsError('QA never overwrites an existing model package: ' + str(target))
        self.request('text', id=lines[0]['id'], text=name)
        self.request('text', id=lines[1]['id'], text=str(target))
        rows = self.snapshot()
        save = next(r for r in rows if r['class'] == 'QPushButton' and r.get('parent') == dialog['id']
                    and r.get('text') == '&Save')
        self.request('click', id=save['id'])
        while time.monotonic() < deadline:
            if (target / 'model.sdf').is_file() and (target / 'model.config').is_file():
                return target
            time.sleep(.05)
        raise AssertionError('Native editor did not create both model files')

    def exit_editor(self, kind):
        rows = self.snapshot()
        action = next(r for r in rows if r['class'] == 'QAction' and r.get('text') == 'E&xit ' + kind + ' Editor')
        self.request('trigger', id=action['id'])
        deadline = time.monotonic() + 5
        while time.monotonic() < deadline:
            rows = self.snapshot()
            buttons = [r for r in rows if r.get('visible') and r['class'] == 'QPushButton' and r.get('text') == 'Exit']
            if buttons:
                self.request('click', id=buttons[0]['id'])
                return
            action = next(r for r in rows if r['class'] == 'QAction' and r.get('text') == '&' + kind + ' Editor')
            if not action.get('checked'):
                return
        raise AssertionError('Native editor exit did not complete')
