from bitstar.state import State, distance
from bitstar.graph import Vertex, Edge, Graph



from bitstar.env import Environment2d

env = Environment2d(0, 10, 0, 10)

print(env.is_free(5, 5))        # should be True
print(env.is_free(-1, 5))       # should be False
print(env.is_free(11, 5))       # should be False
print(env.line_is_free((1,1), (9,9)))   # should be True
print(env.line_is_free((-1,1), (2,2)))  # should be False
