---
name: Bug report
about: Create a report to help us improve
title: "[BUG]"
labels: ''
assignees: ''

---

**Describe the bug**
A clear and concise description of what the bug is.

**Bug report**
Please use the ``camera-bug-report`` tool to create a bug report, and upload it here.

The bug report tool uses the following syntax:

```
camera-bug-report -t <timeout in seconds> -o <output file> -c "<libcamera-app command>"
```

For example,

```
camera-bug-report -t 5 -o bug.txt -c "libcamera-still -t 1000 -o test.jpg"
```
will attempt to run libcamera-still and timeout after 5 seconds if the application has stalled. The script will generate a ``bug.txt`` file that captures all the output logs and system state to help us debug the issue.

You can also run without a timeout:

```
camera-bug-report -o bug.txt -c "libcamera-vid -t 0 -o test.264"
```
This will run ``libcamera-vid`` indefinitely until either you press ``Ctrl+C`` or the application terminates, after which the necessary output logs and system state will be captured.

If you cannot run your application through the ``camera-bug-report`` tool, run it without the ``-c`` command line argument **after running the camera application.**  In these cases, please also provide the command line used to run the application, as well as any output generated during the run.

**Additional context**
Add any other context about the problem here.
