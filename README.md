# Emergency Traffic Custom Discrete-Event Simulator

A discrete-event simulation of wireless nodes written in C. Packets are either best effort or emergency, and emergency packets get a shorter contention window. The program compares a baseline mode with a proposed QoS-adaptive mode. It models packet queues, a contention window, and transmit, carrier-sense and interference ranges over a 20 second simulated run. A small MATLAB file is included.

## Files
- cnlabCcode.c - the simulator.
- cnlabCcode.m - MATLAB file.
