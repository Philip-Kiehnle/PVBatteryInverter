#!/usr/bin/python3
import argparse
import os
import sys

codefile_name = '../STM32_Code/STM32_PVBatteryInverter/Core/Inc/monitoring.h'

START_MARKER = 'typedef struct {'
END_MARKER = '} __attribute__((__packed__)) monitor_vars_t;'

lines_pre = ""
lines_post = ""

def parse_var(line):
    varname = line.lstrip().split(' ')[1].strip().strip(';')
    pre = ''
    post = ''
    if line.find('uint16_t') >= 0 or line.find('uint8_t') >= 0:
        pre = 'unsigned('
        post = ')'
    elif line.find('int16_t') >= 0 or line.find('int8_t') >= 0:
        pre = 'signed('
        post = ')'
    print('cout << "\\n{}=" << {}mon_packet.monitor_vars.{}{};'.format(varname, pre, varname, post))


with open(codefile_name, 'r') as codefile:

    startmarker_found = False
    endmarker_found = False

    for line in codefile:
        # read first section
        if (startmarker_found == False):
            lines_pre += line
            if line.find(START_MARKER) >= 0:
                startmarker_found = True
                print('found START_MARKER', START_MARKER)
        else:
            # read last section
            if line.find(END_MARKER) >= 0:
                endmarker_found = True
                print('found END_MARKER', END_MARKER)
                lines_post += line
            elif endmarker_found:
                lines_post += line

            if (not endmarker_found):
                parse_var(line)

    if startmarker_found == False:
        print('NOT found START_MARKER', START_MARKER)
        sys.exit()

    if endmarker_found == False:
        print('NOT found END_MARKER', END_MARKER)
        sys.exit()
