from path_planner import PathPlanner
p = PathPlanner()
print('initial commands len', len(p.commands))
p.move_to(175,50,45, add_command=True)
print('after move commands len', len(p.commands))
print('last command:', p.commands[-1])
print('last history:', p.path_history[-1])
print('current pose:', p.x, p.y, p.heading)
