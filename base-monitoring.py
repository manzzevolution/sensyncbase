#!/usr/bin/python

import os
import subprocess
import sys

def checkBase():
    try:
        ps = subprocess.check_output('ps -ef | grep sensyncbase_v1.31.py | grep -v grep', shell = True)
        # ps = subprocess.check_output('ps -ef | grep sensyncbase | grep -v grep')
        print ps
        return "ok"
    except:
        return "error"

result = checkBase()
if result == "ok":
    print "base ok"
else:
    print "base error, restarting system"
    os.system("reboot")

