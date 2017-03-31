# pacman
I have not yet implemented the searches in multiagent search. 


Berkeley AI Pacman Challenges
[http://ai.berkeley.edu/project_overview.html]

Problem 1: Finding a fixed food dot using DFS
```
python pacman.py -l mediumMaze -p SearchAgent
python pacman.py -l bigMaze -z .5 -p SearchAgent
```

Problem 2: Finding a fixed food dot using BFS
```
python pacman.py -l mediumMaze -p SearchAgent -a fn=bfs
python pacman.py -l bigMaze -p SearchAgent -a fn=bfs -z .5
```

Problem 3: Finding a fixed food dot using UCS
```
python pacman.py -l mediumMaze -p SearchAgent -a fn=ucs
python pacman.py -l mediumScaryMaze -p StayWestSearchAgent
```

Problem 4: A* search
```
python pacman.py -l tinyCorners -p SearchAgent -a fn=bfs,prob=CornersProblem
python pacman.py -l mediumCorners -p SearchAgent -a fn=bfs,prob=CornersProblem
```

Problem 5: Finding minimal path to reach all corners
Uses A* search and a heuristic function I implement
```
python pacman.py -l mediumCorners -p AStarCornersAgent -z 0.5
```

Problem 6: Find minimal path to eat all the food
Uses A* search and a heuristic function I implement
```
python pacman.py -l trickySearch -p AStarFoodSearchAgent
```