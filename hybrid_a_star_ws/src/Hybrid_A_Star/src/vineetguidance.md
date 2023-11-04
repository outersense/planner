repackage pos_goal publisher and couple curr and goal pose in one and open them when reading tis will ensure taht the timestamps are consistent
update only the obstacle topic, dont publish map at 20hz
take the update map and put it in flow.cpp
or dumb way ( first trial) to run a counter and update map in flow every 10 or 20 iterations
so creatinga  amp is an expensive operation which can slow down the planning package and we want to avoid this 
