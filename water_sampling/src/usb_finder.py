#!/usr/bin/env python
# coding: utf8

import re
import subprocess

class USB:
    def find(id):
        device_re = re.compile(
            "Bus\s+(?P<bus>\d+)\s+Device\s+(?P<device>\d+).+ID\s(?P<id>\w+:\w+)\s(?P<tag>.+)$", re.I)


        df = subprocess.check_output("lsusb")
        devices = []
        for i in df.split('\n'):
            if i:
                info = device_re.match(i)
            if info:
                dinfo = info.groupdict()
                dinfo['device'] = '/dev/bus/usb/%s/%s' % (
                    dinfo.pop('bus'), dinfo.pop('device'))
                device = dinfo['tag']
                if id in device:
                    tags = dinfo['device'].split("/")
                    tag = list(tags[-1])
                    
                    return tag[-1]

        return None
