load map1 % load map
goal = [50,30];
start=[20,10];
ds = Dstar(map); % create navigation object
ds.plan(goal) % create plan for specified goal
ds.query(start) % animate path from this start location