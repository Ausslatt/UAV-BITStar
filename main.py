from bitstar.bit_star import Vertex, bit_star, Tree

start = Vertex(5.0, 5.0)
goal = Vertex(80.0, 80.0)
tree = Tree(start, goal, radius=3.0)
bit_star_planner = bit_star(tree, n_iters=100, sample_size=200)
path = bit_star_planner.planning()
print("Planned path:", path)
