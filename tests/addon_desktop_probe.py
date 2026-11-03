"""Drive the real Qt lesson entry with private settings, data and owned processes."""
import argparse
import json
from pathlib import Path
import sys
import tempfile
import time

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))


def main():
    from PyQt5 import QtCore, QtWidgets
    from gazeboteach.app import create_application
    from gazeboteach.storage import Storage
    from gazeboteach.runtime import _token_processes
    parser=argparse.ArgumentParser()
    parser.add_argument('--lesson',required=True)
    parser.add_argument('--profile',default='classic11-noetic')
    parser.add_argument('--output',required=True)
    parser.add_argument('--timeout',type=float,default=210)
    options=parser.parse_args()
    target=Path(options.output).resolve();target.parent.mkdir(parents=True,exist_ok=True)
    base=Path(tempfile.mkdtemp(prefix='gazeboteach-addon-desktop-'))
    QtCore.QSettings.setDefaultFormat(QtCore.QSettings.IniFormat)
    QtCore.QSettings.setPath(QtCore.QSettings.IniFormat,QtCore.QSettings.UserScope,str(base/'settings'))
    application,window=create_application(ROOT,storage=Storage(base/'data'))
    application.setQuitOnLastWindowClosed(False)
    window.autostart.blockSignals(True);window.autostart.setChecked(False);window.autostart.blockSignals(False)
    window.show()
    report={'ok':False,'lesson':options.lesson,'profile':options.profile,'data_dir':str(base),'checks':[]}
    started=time.monotonic();phase=['environment'];owned=[None]
    def passed(name,**details):report['checks'].append({'name':name,'passed':True,**details})
    def fail(error):
        report['error']=type(error).__name__+': '+str(error)
        if window.session:report['runtime_log']=str(window.session.log_path)
        phase[0]='closing'
        # Only application-owned modal dialogs are closed; no external windows.
        for child in window.findChildren(QtWidgets.QMessageBox):child.reject()
        window.close()
    def tick():
        try:
            if time.monotonic()-started>options.timeout:
                raise RuntimeError('Desktop lesson probe exceeded wall-clock deadline in '+phase[0])
            if phase[0]=='environment' and window.environment is not None:
                if window.environment_dialog and window.environment_dialog.isVisible():window.environment_dialog.accept()
                assert window.select_profile(options.profile),'Profile not found'
                assert window.select_lesson(options.lesson,profile=options.profile),'Lesson not found'
                assert window.buttons['start'].isEnabled(),window.session_status.text()
                passed('real_desktop_material_selected',document_characters=len(window.document.toPlainText()))
                window.buttons['start'].click();phase[0]='starting'
            elif phase[0]=='starting' and not window.busy:
                session=window.session;owned[0]=session
                assert session and session.running,window.session_status.text()
                assert session.health()['gui_running']
                assert session.health()['terminal_running']==2
                passed('start_button_initialized_real_world_gui_and_ptys',session=str(session.session_dir),token=session.token)
                assert window.grab().save(str(target.with_suffix('.png')),'PNG')
                window.buttons['verify'].click();phase[0]='verifying'
            elif phase[0]=='verifying' and not window.busy:
                session=owned[0]
                assert window._last_verification and window._last_verification[0]==session.token
                assert window._last_verification[1]['passed'] is True,window._last_verification
                passed('verify_button_checks_real_behavior')
                phase[0]='closing';window.close()
            elif phase[0]=='closing' and not window.isVisible():
                session=owned[0] or window.session
                assert not session or not _token_processes(session.token),'Owned process remained after close'
                if 'error' not in report:
                    passed('desktop_exit_cleans_owned_processes');report['ok']=True
                application.quit()
        except Exception as error:
            if phase[0]=='closing':
                report['error']=type(error).__name__+': '+str(error)
                if owned[0]:owned[0].stop(reason='Desktop probe failed cleanup')
                application.quit()
            else:fail(error)
    timer=QtCore.QTimer();timer.timeout.connect(tick);timer.start(150)
    try:application.exec_()
    finally:
        if window.session:window.session.stop(reason='附加专项桌面验收结束')
        report['elapsed_seconds']=round(time.monotonic()-started,3)
        target.write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
        print(json.dumps(report,ensure_ascii=False,indent=2))
    return 0 if report['ok'] else 1

if __name__=='__main__':sys.exit(main())
