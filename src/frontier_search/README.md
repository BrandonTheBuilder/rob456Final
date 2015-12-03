## Explanation from paper

Frontier navigation looks to see where in the

world we have explored and tries to find known points that are on the very edge of what we 

know. Those points are known as frontiers. Our node subscribes to the ROStopic /map which 

returns a gmapping occupancy grid of the world, assigning weights to the cells in the grid based 

on if they are occupied/not occupied or unknown. The first thing that the algorithm does is to 

look through the map for any clear cells where the cell value is ‘0’. It will then look through the 

list of cleared cells in the grid and check to see if they have any neighboring cells that are 

unknown with cell values of ‘-1’. All the cells that remain are the frontiers, (clear cells 

neighboring other cells with unknown values). We also applied a filter that removed any 

frontiers within a given distance of an obstacle to try to keep the robot from getting stuck. 

Once we had a list of frontiers we needed a find an optimal waypoint that was easy for the 

robot to reach and explored the most area. We accomplished this by applying a cost heuristics 

to each of the frontiers. Our first heuristic is simply the length of the Manhattan distance from 

our robot to the frontier, we want to explore any node that is close to us first. The second 

heuristic is the Manhattan distance from the frontier to the average value of all frontiers, this 

finds a frontier that is closest to the largest concentration of other frontiers giving it a higher 

exploration value. We set two scaling factors for each of these heuristics to try and get a 

balance between easy to reach frontiers and frontiers that will allow for exploring more 

territory. We then set the frontier with the lowest cost as our waypoint. 

Once we decide which frontier to travel too we needed to send the goal to move base and then 

wait for our robot to reach it at which point we check for new frontiers and repeat the process 

until there are no new frontiers left.