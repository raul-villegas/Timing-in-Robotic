# Timing-in-Robotic (UBUNTU)
For the pthread:
compile
g++ -fext-numeric-literals pthread.cpp -lpthread -o pthread 

run
./pthread

stress --cpu 2

htop

For myprogram:
The code uses the Xenomai-specific rt_task_create() and rt_task_start() functions to create and start a real-time task. The task is set to have a default priority and mode, but this can be adjusted as needed.
The periodic_task() function performs some simple computational work (in this case, calculating the sine of the loop index) and then sleeps for a fixed period of 1 ms using clock_nanosleep().
The code also uses the clock_gettime() function to get a precise timestamp for each loop iteration, which can be used to calculate the jitter.
To compile the code for Xenomai, you will need to use the xeno-config utility to get the required compiler and linker options. Here's an example compilation command:

g++ -o myprogram myprogram.cpp `xeno-config --skin=native --cflags --ldflags`

To run the program, you will need to first start the Xenomai real-time environment using the xenomai command, and then run the program using sudo to ensure that it has the necessary privileges:

sudo xenomai
sudo ./myprogram


