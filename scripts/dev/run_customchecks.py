#!/usr/bin/env python3

import json
import subprocess
import sys

custom_checks_to_run = [
    "scripts/dev/license-check.py",
    "scripts/dev/verify_formatting.sh",
    "scripts/dev/license-check.py",
    "scripts/dev/check_stray_fields_in_idd.py",
    "scripts/dev/verify_idfs_in_cmake.py",
    "scripts/dev/check_non_utf8.py",
    "scripts/dev/check_pr_labels.py",
    "scripts/dev/verify_file_encodings.py",
    "scripts/dev/validate_idd_units.py",
    "scripts/dev/find_byref_bool_override.py",
    "scripts/dev/check_for_tabs_in_idfs.py",
    "scripts/dev/check_for_bom_in_idfs.py"
]


def parse_custom_check_run_output(output):
    # any output from a custom_check indicates an error
    any_errors_this_run = False
    for line in output.split('\n'):
        if line == '':
            continue
        any_errors_this_run = True
        try:
            json_object = json.loads(line)
        except json.decoder.JSONDecodeError:
            print("Error processing custom_check output line, expected it to be a single line JSON object")
            print("Actual line is: \"" + line + "\"")
            return
        if isinstance(json_object, list):
            print("Error in custom_check output line, encountered array, expected single line JSON object")
            print("Actual line is: \"" + line + "\"")
            return
        tool = json_object.get('tool', None)
        file = json_object.get('file', '(Unknown file)')
        line_num = json_object.get('line', -1)
        column = json_object.get('column', None)
        messagetype = json_object.get('messagetype', 'error')  # defaults to error
        message = json_object.get('message', '(No message)')
        id = json_object.get('id', None)
        # make the message nicer if possible
        if id:
            message = str(id) + message
        if tool:
            message = str(tool) + message
        location = "%s:%s" % (file, line_num)
        if column:
            location += ':' + column
        print(" ** [%s] %s - @ %s" % (messagetype, message, location))
    return any_errors_this_run


def run_all_custom_checks(_build_dir):
    any_errors_overall = False
    for script in custom_checks_to_run:
        print("Running CustomCheck: " + script)
        output = subprocess.run([script, _build_dir], stdout=subprocess.PIPE).stdout
        output = output.decode("utf-8")
        if output == "":
            continue
        any_errors_this_run = parse_custom_check_run_output(output)
        if any_errors_this_run:
            any_errors_overall = True
    return any_errors_overall


if len(sys.argv) > 1:
    build_dir = sys.argv[1]
else:
    build_dir = ""
any_errors = run_all_custom_checks(build_dir)
if any_errors:
    sys.exit(1)

