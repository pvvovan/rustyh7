import xml.etree.ElementTree as ET
import sys
import os
import pathlib

class Component:
    Comment = 'Comment'
    Designator = 'Designator'
    Footprint = 'Footprint'
    PartN = 'JLCPCB Part#'


tree = ET.parse(sys.argv[1])
root = tree.getroot()

all_components = []

for el in root[1]:
    component = Component()
    fields = el.findall('fields')
    if len(fields) == 1 and fields[0][0].attrib['name'] ==  'JLC#':
        component.PartN = fields[0][0].text
    elif len(fields) == 0:
        component.PartN = ''
    else:
        print('JLCPCB part number not found')
        quit(1)
    component.Comment = el[0].text
    component.Designator = el.attrib['ref']
    component.Footprint = el[1].text
    all_components.append(component)

# remove components which do not have JLCPCB part number
jlc_components = []
for el in all_components:
    if el.PartN != '':
        jlc_components.append(el)

# check JLCPCB part number has the same values and footprints in all components
for ref_el in jlc_components:
    for el in jlc_components:
        if ref_el.PartN == el.PartN:
            if ref_el.Comment != el.Comment:
                print('Values do not match')
            elif ref_el.Footprint != el.Footprint:
                print('Footprints do not match')
            else:
                continue
            print(ref_el.Designator + ' ' + el.Designator)
            quit(1)


# generate position file
dir_asm = os.path.dirname(sys.argv[1]) + '/assemble'
files_asm = os.listdir(dir_asm)

if len(files_asm) != 1:
    print('Number of files in directory assemble is not 1')
    quit(1)

filename_asm = dir_asm + '/' + files_asm[0]
file_extension = pathlib.Path(filename_asm).suffix
if file_extension != '.csv':
    print('csv file not found')
    quit(1)

file_asm = open(filename_asm, 'r')
file_asm.readline()
file_pos = open(sys.argv[2] + '_jlcpos.csv', 'w')
file_pos.write('Designator,Mid X,Mid Y,Layer,Rotation')

while True:
    asm_line = file_asm.readline()
    if not asm_line:
        break
    asm_elems = asm_line.split(',')
    asm_designator = asm_elems[0][1:-1]
    designator_exists = False
    for jlc_component in jlc_components:
        if jlc_component.Designator == asm_designator:
            designator_exists = True
            break
    if designator_exists:
        file_pos.write('\r\n')
        file_pos.write(asm_designator + ',')
        file_pos.write(asm_elems[3] + ',')
        file_pos.write(asm_elems[4] + ',')
        if asm_elems[6][0:-1] == 'top':
            file_pos.write('Top,')
        elif asm_elems[6][0:-1] == 'bottom':
            file_pos.write('Bottom,')
        else:
            print('Layer not supported')
            quit(1)
        file_pos.write(asm_elems[5])

file_pos.close()
file_asm.close()


# generate BOM file
sorted_components = sorted(jlc_components, key=lambda el: el.PartN)
bom = []
is_first = True
for sc in sorted_components:
    if is_first:
        is_first = False
        bom.append(sc)
    elif bom[-1].PartN == sc.PartN:
        bom[-1].Designator = bom[-1].Designator + ', ' + sc.Designator
    else:
        bom.append(sc)

quoted_bom = [];
for el in bom:
    quoted_bom.append(el)
    quoted_bom[-1].Designator = '"' + quoted_bom[-1].Designator + '"'

sorted_bom = sorted(quoted_bom, key=lambda el: el.Designator)

file_bom = open(sys.argv[2] + '_jlcbom.csv', 'w')
s = "Comment,Designator,Footprint,JLCPCB Part#"
file_bom.write(s)

for component in sorted_bom:
    file_bom.write('\r\n')
    file_bom.write(component.Comment)
    file_bom.write(',')
    file_bom.write(component.Designator)
    file_bom.write(',')
    file_bom.write(component.Footprint)
    file_bom.write(',')
    file_bom.write(component.PartN)

file_bom.close()
