# Headtracker_Console

Here is a quick video of the program being used in the Game IL 2 - Battle of Stalingrad

This video uses 4 points to determine headpose. 

https://github.com/user-attachments/assets/c318f381-c0a2-4335-9592-3b1a7a840bda


Here is an earlier video which used 3 points. The game here is war thunder.

https://github.com/user-attachments/assets/2aa78c79-7335-4bd5-b44b-7dcee232e18d

It must be noted that this headtracker sends the head pose data to "OpenTrack" which can be found here https://github.com/opentrack/opentrack

The head tracker is only used to determine head pose. Whats the head pose has been found, the data is send to opentrack via udp and from there, open track smoothes out the values and send the smoothed values to the game.

Before are the some pictures of what my hardware looks like. It is merely a prototype so dont think much of it.

![headtracker github](https://github.com/user-attachments/assets/08710f35-a50f-4a03-bb94-71cba078ee2f)

The 3 point version will simply have thesame 2 base leds with one top led in the center.






