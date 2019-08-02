import sys
import os

def process(line, pose, outfile):
	l = line.split()
	if len(l) == 6:
		if pose:
			outfile.write(' '.join(l) + "\n")
			# print(line)
		return not pose
	return pose

names = os.listdir("new_data/")

for file in names:
	start = 0
	pose = False
	new_file  = open("pose_data/" + file, 'w')
	with open("new_data/" + file) as f:
	    for line in f:
	    	if start < 3:
	    		if line[0] == '*':
		    		start += 1
		    	new_file.write(line)
	    	else:
	        	pose = process(line, pose, new_file)
	f.close()
	new_file.close()
