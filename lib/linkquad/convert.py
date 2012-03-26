#!/usr/bin/python
import re;
infile = "conversion.hpp"
outfile = "converted.hpp"
pattern = r'^(?P<tabs>[ ]+)(?P<type>[^ ]+) (?P<name>[^\[;]+)(\[(?P<n>[0-9N]+)\])?;(?P<i>0)?$'

i = 0
def repl(m):
    global i
    if m.group('type') == 'typedef':
        return m.group(0)

    if m.group('i') is not None:
        i = int(m.group('i'))

    name = m.group('name')
    if m.group('n') is not None and m.group('n') is not "N":
        iterate = int(m.group('n'))
    else:
        iterate = 1

    iteration = 0
    result = ""
    while iteration < iterate:
        nname = name + ("_" + str(iteration) if iterate > 1 else '')
        result += m.group('tabs') + "struct " + nname + "_t { typedef " \
            + m.group('type') + " type" + ("[N]" if m.group('n') == "N" else "") + "; type " + nname + "; static const size_t size = sizeof(type); " \
            + "static const uint8_t c = 1; static const int n = " + str(i) + "; " \
            + "void set(const uint8_t*& v){memcpy(&" + nname + ", v, sizeof(type)); v += sizeof(type); } " \
            + "void get(uint8_t*& v) const {memcpy(v, &" + nname + ", sizeof(type)); v += sizeof(type); } }; " \
            + ("typedef " + nname + "_t " + nname + ";" if m.group('n') != "N" else "") + ("\n" if iteration+1 < iterate  else "")
        i += 1
        iteration += 1
    #~ if iterate > 1:
        #~ result += m.group('tabs') + "struct " + name + "_t { typedef " \
            #~ + m.group('type') + " type[" + str(iterate) + "]; type " + name + "; static const size_t size = sizeof(type); " \
            #~ + "static const uint8_t c = " + str(iterate) + "; static const int n[] = {" + ",".join("%s" % num for num in range(i-iterate, i)) + "}; " \
            #~ + "void set(const uint8_t*& v){memcpy(&" + name + ", v, sizeof(type)); v += sizeof(type); } " \
            #~ + "void get(char*& v){memcpy(v, &" + name + ", sizeof(type)); v += sizeof(type); } }; " \
            #~ + "typedef " + name + "_t " + name + ";\n"
    return result


with open(infile, 'r') as f:
    with open(outfile, 'w') as o:
        for line in f:
            o.write(re.sub(pattern, repl, line))
