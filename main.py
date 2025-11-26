from bitstar.state import State, distance
from bitstar.graph import Vertex, Edge, Graph


s1 = State(0, 0)
s2 = State(0, 4)
s3 = State(3, 0)

print s1
print s2
print distance(s1, s2)




g = Graph()

g.add_vertex(s1)
g.add_vertex(s2)
g.add_vertex(s3)


g.add_edge(0,1)
g.add_edge(1,2)
g.add_edge(2,0)

print g.num_vertices()
print g.num_edges()

print g.get_vertex(0)
print g.get_vertex(1)
print g.get_vertex(2)
