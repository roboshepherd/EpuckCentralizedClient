#!/usr/bin/env python

import os, signal, subprocess, re, sys

numargs = len(sys.argv) 
if( numargs < 2):
    print "Usage %s : robotid" %sys.argv[0]
    sys.exit(1)
else:
    robotid = sys.argv[1]	

cmd = "ps aux | grep ClientMain.py"
subproc = subprocess.Popen([cmd, ], stderr=subprocess.PIPE, stdout=subprocess.PIPE, shell=True)
out = subproc.communicate()
#print "\t ps says:" + out[0]
#print "\n"
lines=  out[0].split("\n")
#print "\t Output lines: \n", lines
for line in lines:
	print "Line:-->", line
	output = line.split()
	try:
		if (output[12] == robotid):
			print "Killing PID:", output[1]
			pid = int(output[1])
			if(pid > 0):
				os.kill(pid, signal.SIGKILL)	    
		else:
			print "PID not OK"
	except Exception, e:
		print e
	# Killing valid PID	

