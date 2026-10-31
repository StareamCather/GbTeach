#!/usr/bin/env python3
"""Finite actual failure/repair verification; evidence is retained in build/."""
import json
from clinic import run
from lab import Lab


if __name__=='__main__':
    preview=Lab().ready()
    result=run()
    print(json.dumps({'passed':result['passed'],'report_file':result['report_file'],
                      'cases':len(result['cases']),'preview_ready':preview['ready']},ensure_ascii=False))
