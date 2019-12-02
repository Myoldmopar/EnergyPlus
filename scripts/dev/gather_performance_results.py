#!/usr/bin/env python3

# run from the performance_tests directory inside the build directory after performance tests have been executed

import json
import sys
import os
import glob
import re


def get_name(files, id, name):
    if not name:
        return files, files[id]
    elif not id:
        return files, name
    else:
        files[id] = name
        return files, name


def parse_call_grind_file(content):

    object_file = None
    source_file = None
    call_count = None
    called_object_file = None
    called_source_file = None
    called_function_name = None
    called_functions = []
    object_files = {}
    source_files = {}
    functions = {}
    props = {}

    lines = content.split("\n")
    re_field = re.compile(r'^(?P<field>[a-z]+): (?P<data>.*)')
    re_ob = re.compile(r'^ob=(?P<objectfileid>\([0-9]+\))?\s*(?P<objectfilename>.*)?')
    re_fl = re.compile(r'^fl=(?P<sourcefileid>\([0-9]+\))?\s*(?P<sourcefilename>.*)?')
    re_fe_fi = re.compile(r'^(fe|fi)=(?P<sourcefileid>\([0-9]+\))?\s*(?P<sourcefilename>.*)?')
    re_fn = re.compile(r'^fn=(?P<functionid>\([0-9]+\))?\s*(?P<functionname>.*)?')
    re_cob = re.compile(r'^cob=(?P<calledobjectfileid>\([0-9]+\))?\s*(?P<calledobjectfilename>.*)?')
    re_cfi_cfl = re.compile(r'^(cfi|cfl)=(?P<calledsourcefileid>\([0-9]+\))?\s*(?P<calledsourcefilename>.*)?')
    re_cfn = re.compile(r'^cfn=(?P<calledfunctionid>\([0-9]+\))?\s*(?P<calledfunctionname>.*)?')
    re_calls = re.compile(r'^calls=(?P<count>[0-9]+)?\s+(?P<target_position>[0-9]+)')
    re_subpos = re.compile(r'^(?P<subposition>(((\+|-)?[0-9]+)|\*)) (?P<cost>[0-9]+)')
    num_lines = len(lines)
    print_interval = round(num_lines / 10)
    line_num = 0
    for line in lines:
        line_num += 1
        if line_num / print_interval == round(line_num / print_interval):
            print(" ... On line %i out of %i (%i%%)" % (line_num, num_lines, (100 * line_num / num_lines)))
        # quick break for blank lines
        if line == '':
            continue
        # then start searching for each possible regex match
        re_field_match = re_field.search(line)
        re_ob_match = re_ob.search(line)
        re_fl_match = re_fl.search(line)
        re_fe_fi_match = re_fe_fi.search(line)
        re_fn_match = re_fn.search(line)
        re_cob_match = re_cob.search(line)
        re_cfi_cfl_match = re_cfi_cfl.search(line)
        re_cfn_match = re_cfn.search(line)
        re_calls_match = re_calls.search(line)
        re_subpos_match = re_subpos.search(line)
        # do the right thing based on the result
        if re_field_match:
            field = re_field_match.group('field')
            data = re_field_match.group('data')
            props[field] = data
        elif re_ob_match:
            objectfileid = re_ob_match.group('objectfileid')
            objectfilename = re_ob_match.group('objectfilename')
            object_files, object_file = get_name(object_files, objectfileid, objectfilename)
        elif re_fl_match:
            sourcefileid = re_fl_match.group('sourcefileid')
            sourcefilename = re_fl_match.group('sourcefilename')
            source_files, source_file = get_name(source_files, sourcefileid, sourcefilename)
        elif re_fe_fi_match:
            sourcefileid = re_fe_fi_match.group('sourcefileid')
            sourcefilename = re_fe_fi_match.group('sourcefilename')
            source_files, _ = get_name(source_files, sourcefileid, sourcefilename)
        elif re_fn_match:
            functionid = re_fn_match.group('functionid')
            functionname = re_fn_match.group('functionname')
            functions, _ = get_name(functions, functionid, functionname)
        elif re_cob_match:
            calledobjectfileid = re_cob_match.group('calledobjectfileid')
            calledobjectfilename = re_cob_match.group('calledobjectfilename')
            object_files, called_object_file = get_name(object_files, calledobjectfileid, calledobjectfilename)
        elif re_cfi_cfl_match:
            calledsourcefileid = re_cfi_cfl_match.group('calledsourcefileid')
            calledsourcefilename = re_cfi_cfl_match.group('calledsourcefilename')
            source_files, called_source_file = get_name(source_files, calledsourcefileid, calledsourcefilename)
        elif re_cfn_match:
            calledfunctionid = re_cfn_match.group('calledfunctionid')
            calledfunctionname = re_cfn_match.group('calledfunctionname')
            functions, called_function_name = get_name(functions, calledfunctionid, calledfunctionname)
        elif re_calls_match:
            call_count = re_calls_match.group('count')
        elif re_subpos_match:
            cost = re_subpos_match.group('cost')
            if call_count is not None:
                this_object_file = called_object_file if called_object_file else object_file
                this_source_file = called_source_file if called_source_file else source_file

                index = -1
                for x in called_functions:
                    index += 1
                    if x['this_object_file'] == this_object_file and x['this_source_file'] == this_source_file and x['called_function_name'] == called_function_name:
                        found = True
                        break
                else:
                    found = False
                if not found:
                    called_functions.append(
                        {
                            'this_object_file': this_object_file,
                            'this_source_file': this_source_file,
                            'called_function_name': called_function_name,
                            'count': 0,
                            'cost': 0
                        }
                    )
                    index = len(called_functions) - 1
                called_functions[index]['count'] += int(call_count)
                called_functions[index]['cost'] += int(cost)

                # reset some stuff
                call_count = None
                called_object_file = None
                called_source_file = None
                called_function_name = None

    print(" ... Finished line %i (100%%)" % num_lines)

    props['object_files'] = []

    # we are in the build/performance_tests folder, so the build folder is one dir up
    perf_dir = os.getcwd()
    build_dir = os.path.dirname(perf_dir)

    for this_file in object_files:
        file_path = object_files[this_file]
        if not os.path.exists(file_path) or not file_path.startswith(build_dir):
            continue
        info = os.stat(file_path)
        props['object_files'].append({'name': os.path.relpath(file_path, build_dir), 'size': info.st_size})

    def _cost(item):
        if item['this_object_file'].startswith(build_dir):
            return item['cost']
        return 0

    def _count(item):
        if item['this_object_file'].startswith(build_dir):
            return item['count']
        return 0

    most_expensive = sorted(called_functions, key=_cost, reverse=True)[0:50]
    most_called = sorted(called_functions, key=_count, reverse=True)[0:50]
    most_expensive.extend(most_called)
    important_functions = []
    for function_call in most_expensive:
        important_functions.append({
            'object_file': function_call['this_object_file'],
            # 'source_file': function_call['this_source_file'],
            'function_name': function_call['called_function_name'],
            'count': function_call['count'],
            'cost': function_call['cost']
        })
    props['data'] = important_functions
    return props


def gather_performance_results():
    performance_total_time = 0
    performance_test_count = 0
    results = {'test_files': []}  # 'object_files': []
    call_grind_files = glob.glob('./**/callgrind.*')
    for call_grind_file in call_grind_files:
        print("Working on file: " + call_grind_file)
        performance_test_name = call_grind_file
        word = 'callgrind'
        if word in call_grind_file:
            performance_test_name = call_grind_file.split(word)[1]
        with open(call_grind_file) as f:
            call_grind_output = parse_call_grind_file(f.read())
        # object_files = call_grind_output.pop('object_files')
        # results['object_files'].append(object_files)
        call_grind_output['test_name'] = performance_test_name
        results['test_files'].append(call_grind_output)
        performance_test_count += 1
        if call_grind_output['totals']:
            performance_total_time += int(call_grind_output['totals'])
    return results


try:
    performance_results = gather_performance_results()
    print(json.dumps(performance_results, indent=2))
    with open('performance.json', 'w') as f:
        f.write(json.dumps(performance_results, indent=2))
except:
    sys.exit(1)
