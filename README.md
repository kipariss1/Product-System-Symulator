# Product-System-Symulator
Simulation of pneumatic cylinder, controlled by PLC. Simulation is made with Python 3.7.3 on Raspberry Pi v 1.2 with UniPi v1.1 extencion board, connected to PLC.

The simulator is monostable_graph_rpi.py. File konfiguration.txt is configuration file, which shows current parameters of simulation. You can change the parameters of simulation by changing the values of the parameters in konfiguration.txt.

*Running the simulation:

You should download monostable_graph_rpi.py and konfiguration.txt in same repository and run the python file. Simulation is meant to be executed on Raspberry Pi with UniPi v1.1 attached and RPi+UniPi connected to PLC. With this setup it will simulate the working cycle of loaded pneumatic cylinder with monostable direct control valve and two end switchers, which will be tracking end positions of piston of the cylinder. Simple visualisation, made with pygame and graph, which shows the position of the piston will be displayed during simulation. To stop the simulation you should use KeyboardInterrupt exeption (press Ctrl+C).

*Testing the simulation:

However you can test simulation on RPi, without UniPi extension board or PLC and you can even test in it on PC. There is built-in simulation of states of pins on RPi for testing on PC. So you can just run the simulation of pneumatic cylinder with visualisation and graph of the piston position without attaching any peripheral devices to your RPi or even on PC. 

In the repository you can find peumatic scheme of simulated system (scheme_of_monostable_task_2.png), sequential block scheme of technological process (block_scheme_mono.png), scheme of the tehcnological process (mono.png) and electrical circuit scheme of connection of the hardware for running the simulation (circuit_mono_sim.png).
