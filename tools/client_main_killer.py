#!/usr/bin/env python

import os, signal, subprocess, re, sys

cmd = "ps aux | grep ClientMain.py"
subproc = subprocess.Popen([cmd, ], stderr=subprocess.PIPE, stdout=subprocess.PIPE, shell=True)
out = subproc.communicate()
#print "\t ps says: \n" + out[0]
lines=  out[0].split("\n")
#print "\t Output lines: \n", lines
for line in lines:
	print "Line:-->", line
	output = line.split()
	try:
		if (output[11] != 'ClientMain.py'):
			print "server not killed"
			continue
		else:
			print "Killing PID:", output[1]
			pid = int(output[1])
	except Exception, e:
		print e
	# Killing valid PID	
	if(pid > 0):
	    try:
		os.kill(pid, signal.SIGKILL)
	    except Exception, e:
		print e
	else:
		print "PID not OK"
