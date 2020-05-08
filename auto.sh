# Star here: What is a shell?
# A command-line interpreter.
# It provides a command line user interface for Unix-like operating systems. 

# Basicly a .sh script allows you to bundle together commands that will be 
# executed by the operating system. 
# Useful for automating jobs as a result. 



#!/bin/bash
# declare STRING variable

counter=1
for i in 1 2 3 4 5

do
./waf --run "scratch/loaded_four --duration=285 --tcp_algorithm=TcpYeah --run=$i"
echo $counter
((counter++))
done

echo All done




#Note: May need to add matlab and the directory you are working in 
# to the search path for the shell

# Add matlab to the Shell PATH: 
# export PATH=$PATH:/Applications/MATLAB_R2019a.app/bin/

# Add /usr/local/bin to the Shell PATH:
# export PATH=$PATH: /usr/local/bin

