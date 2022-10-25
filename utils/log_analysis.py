#!/bin/python3
import json
import sys
import math
import ctypes
import sys

#{"hspec": 0.195, "lspec": 0.175, "name": "whiteArea_fraction-B", "status": "PASS", "units": "", "val": 0.182}

contents = []
with open(sys.argv[1], 'rt') as f:
    contents = f.readlines()
    contents = [l.rstrip() for l in contents]

j = [json.loads(c) for c in contents];

mode_log = [i['mode_log'] for i in j]

names = []
hspec = []
lspec = []
number = []
sum = []
sos = []
mins = []
maxs = []
nfail = []
noob = []


for d in mode_log[0]:
    names.append(d['name'])
    hspec.append(d['hspec'])
    lspec.append(d['lspec'])
    number.append(0)
    nfail.append(0)
    noob.append(0)
    sum.append(0.0)
    sos.append(0.0)
    mins.append(d['val'])
    maxs.append(d['val'])

for entry in mode_log:
        i = 0
        for d in entry:
            if names[i] != d['name']:
                i = names.index(d['name'])
            v = d['val']
            number[i] = number[i] + 1
            sum[i] = sum[i] + v
            sos[i] = sos[i] + v*v
            mins[i] = min(v, mins[i])
            maxs[i] = max(v, maxs[i])
            if d['status'] != 'PASS':
                nfail[i] += 1
            if v > hspec[i] or v < lspec[i]:
                noob[i] += 1
            i += 1

avg = []
sd = []

print('name                         lspec       min        avg        max       hspec   fail%           v-lspec                           hspec-v')
for i in range(len(number)):
    avg.append(sum[i] / number[i])
    sd.append(math.sqrt(max(0.0, sos[i]/number[i] - avg[i]*avg[i])))
    foostr = ""
    for z in range(64):
        lo = lspec[i] + (z-12)*(hspec[i]-lspec[i])/40.0;
        hi = lspec[i] + (z-11)*(hspec[i]-lspec[i])/40.0;
        if mins[i] >= lo and maxs[i] < hi:
            foostr += 'I'
        elif mins[i] >= lo and mins[i] < hi:
            foostr += '['
        elif maxs[i] >= lo and maxs[i] < hi:
            foostr += ']'
        elif avg[i] >= lo and avg[i] < hi:
            foostr += '+'
        elif avg[i]-sd[i] >= lo and avg[i]-sd[i] < hi:
            foostr += '<'
        elif avg[i]+sd[i] >= lo and avg[i]+sd[i] < hi:
            foostr += '>'
        elif avg[i]+sd[i] >= lo and avg[i]-sd[i] < hi and maxs[i] >= lo and mins[i] < hi:
            foostr += '='
        elif maxs[i] >= lo and mins[i] < hi:
            foostr += '-'
        else:
            foostr += '|' if z==12 or z==52 else ' '
    print(f'{names[i]:24s}{lspec[i]:10g} {mins[i]:10g} {avg[i]:10g} {maxs[i]:10g} {hspec[i]:10g}{100.0*nfail[i]/number[i]:7.2f}{foostr}')

