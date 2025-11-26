from bitstar.state import State, distance


class Vertex(object):
	
	def __init__(self, state, g=float('inf'), h=0.0, parent=None):
		self.state = state
		self.g = g # cost-to-come from start Vertex 
		self.h = h # heuristic value
		self.parent = parent # idx of the parent vertex



class Edge(object):

	def __init__(self, u, v, cost):
		self.u = u
		self.v = v
		self.cost = cost


class  Graph(object): 

	def __init__(self):
		self.vertices = []
		self.edges = []

	def add_vertex(self, state):
		idx = len(self.vertices)
		self.vertices.append(Vertex(state))
		return idx

	def add_edge(self, u_idx, v_idx):
		u_state = self.vertices[u_idx].state
		v_state = self.vertices[v_idx].state
		cost = distance(u_state, v_state)
		e = Edge(u_idx, v_idx, cost)
		self.edges.append(e)
		return e

	def num_vertices(self):
		return len(self.vertices)

	def num_edges(self):
		return len(self.edges)

	def get_vertex(self, idx):
		return self.vertices[idx]

