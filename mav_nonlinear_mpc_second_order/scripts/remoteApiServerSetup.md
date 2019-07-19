--------------------------------------------------
Starting simulation from python script using remote API:
--------------------------------------------------
Useful link:
http://www.forum.coppeliarobotics.com/viewtopic.php?t=537

----------------------------------------------------
TL;DR:
By default, VREP opens a remote API server on port 19997.
If you simply connect your remoteApi client to this port everything should work fine.
----------------------------------------------------

More info:
Usually, users include the following command inside a child script which runs once when the simulation is started to enable external scripts to connect to VREP:

# simRemoteApi.start(19999)

This enables remote API clients to connect to port 19999.
However, this does not allow external scripts to connect to VREP when the simulation has stopped. In order to control VREP or a scene via the remote API while simulation is not running, you need to prepare a "continous remote API server service" that is running as soon as you start VREP.

More information in link below:
http://www.coppeliarobotics.com/helpFiles/en/remoteApiServerSide.htm

In short, in order to start the remote api server on startup:
- in your V-REP installation folder, there is a file called remoteApiConnections.txt.
- Open it and edit it to something like:
    portIndex1_port 		= 19998
    portIndex1_debug 		= true
    portIndex1_syncSimTrigger 	= false
- Then restart V-REP. At start-up, V-REP will start a continuous remote API server service on port 19998, and display a debug window. In the debug window you can see if a connection to a client was successful.
- The client side (e.g. your python script) will have to connect on that port 19998. Refer to the instructions for the client in the link below:
http://www.coppeliarobotics.com/helpFiles/en/remoteApiClientSide.htm

--------------------------------------------------
Running simulations circularly and automatically:
--------------------------------------------------
Useful link: 
http://www.forum.coppeliarobotics.com/viewtopic.php?t=7110

-------------------------------------------------
Command line stuff for unit tests and whatnot:
-------------------------------------------------
http://www.coppeliarobotics.com/helpFiles/en/commandLine.htm
-------------------------------------------------
