
import heapq
import math
import random


class Vertex(object):
	
	def __init__(self,x, y, parent=None):
            """
            Initialize a vertex with coordinates (x, y) and an optional parent vertex.
            :param x: x-coordinate of the vertex
            :param y: y-coordinate of the vertex
            :param parent: parent vertex (default is None)
            """
            self.x = x
            self.y = y
            self.parent = parent 


class Tree(object):
    
    def __init__(self, start, goal, radius=3.0):
        
        """
        Initialize a tree with start and goal vertices, and a connection radius.
        :param start: start vertex
        :param goal: goal vertex
        :param radius: connection radius (default is 3.0)
        """

        self.start = start
        self.goal = goal
        self.radius = radius
        self.V = set()
        self.E = set()
        self.Qe = set()
        self.Qv = set()
        self.V_old = set()



    
class bit_star(object):


    def __init__(self, Tree, n_iters, sample_size=100):
        """ 
        Initialize the BIT* algorithm with a tree, start and goal vertices, and number of iterations.
        :param Tree: tree object containing vertices and edges
        :param n_iters: number of iterations to run the algorithm
        """
        self.Tree = Tree
        self.n_iters = n_iters
        self.sample_size = sample_size

        # Sample set (unconnected states)
        self.x_samples = set()

        # Heaps for priority queues (we keep sets in Tree for membership)
        self._Qv_heap = []   # (key, counter, vertex)
        self._Qe_heap = []   # (key, counter, (v, x))
        self._heap_counter = 0  # tie-breaker


    
    def planning(self):
        self.Tree.V.add(self.Tree.start)
        self.x_samples.add(self.Tree.goal)

        for i in range(self.n_iters):
            if not self.Tree.Qe and not self.Tree.Qv:
                self.prune(self.g_T(self.Tree.goal))
                self.x_samples.update(self.sample_free(self.sample_size))
                self.Tree.V_old = self.Tree.V.copy()
                self.Tree.Qv = self.Tree.V.copy()
                

                self._Qe_heap = []
                self._Qv_heap = []
                self._heap_counter = 0

                for v in self.Tree.Qv:
                    self._enqueue_vertex(v)


                self.Tree.radius = self.calc_radius(len(self.Tree.V) + len(self.x_samples))

            while self.pop_vertex() <= self.pop_edge():
                    v_best = self.best_vertex()
                    if v_best is None:
                         break
                    
                    self.expand_vertex(v_best)
            
            vm, xm = self.best_edge()
            
            if vm is None or xm is None:
                continue

            self.Tree.Qe.remove((vm, xm))

            
                 
        
            if self.g_T(vm) + self.calc_dist(vm, xm) + self.h_hat(xm) < self.g_T(self.Tree.goal):
                if self.g_hat(xm) + self.c_actual(vm,xm) + self.h_hat(xm) < self.g_T(self.Tree.goal):
                    if self.g_T(vm) + self.c_actual(vm,xm) < self.g_T(xm):
                        
                        
                        if xm in self.Tree.V:
                            pruned_edges = set()

                            for v,w in self.Tree.E:
                                if w == xm:
                                    pruned_edges.add((v,w))
                            for edge in pruned_edges:
                                self.Tree.E.remove(edge)
                        else:

                            if xm in self.x_samples:
                                 self.x_samples.remove(xm)
                            self.Tree.V.add(xm)
                            self.Tree.Qv.add(xm)
                            self._enqueue_vertex(xm)

                        
                        self.Tree.E.add((vm, xm))
                        xm.parent = vm
                        
                        bad_edges = set()


            else:
                self.Tree.Qe = set(); self.Tree.Qv = set()
                self._Qe_heap = []; self._Qv_heap = []

        
        return self.get_path()
    

    def expand_vertex(self, v):
        """
        Expands vertex v by adding candidate edges to the edge queue Qe.
        """
        g_goal = self.g_T(self.Tree.goal)

        # Remove v from the vertex queue (set only; heap entries become stale)
        if v in self.Tree.Qv:
            self.Tree.Qv.remove(v)

        # Nearby samples
        X_near = set(
            x for x in self.x_samples
            if self.calc_dist(x, v) <= self.Tree.radius
        )
       

        # Candidate edges from v to nearby samples
        for x in X_near:
            if self.g_hat(v) + self.c_hat(v, x) + self.h_hat(x) < g_goal:
                edge = (v, x)
                if edge not in self.Tree.Qe:
                    self.Tree.Qe.add(edge)
                    self._enqueue_edge(edge)

        # If v has not been expanded before, also consider nearby vertices
        if v not in self.Tree.V_old:
            V_near = set(
                w for w in self.Tree.V
                if self.calc_dist(w, v) <= self.Tree.radius
            )

            for w in V_near:
                edge = (v, w)
                if (edge not in self.Tree.E and
                    self.g_hat(v) + self.c_hat(v, w) + self.h_hat(w) < g_goal and
                    self.g_T(v) + self.c_hat(v, w) < self.g_T(w)):
                    if edge not in self.Tree.Qe:
                        self.Tree.Qe.add(edge)
                        self._enqueue_edge(edge)

    def prune(self, c_best):
        """
        Prunes the tree and samples based on the current best solution cost c_best.
        """
        # Remove samples that cannot possibly be on a better solution
        self.x_samples -= set(
            x for x in self.x_samples if self.f_hat(x) >= c_best
        )

        # Remove vertices that cannot possibly be on a better solution
        self.Tree.V -= set(
            v for v in self.Tree.V if self.f_hat(v) > c_best
        )

        # Remove edges entirely between such bad vertices
        self.Tree.E -= set(
            (v, w) for (v, w) in self.Tree.E
            if self.f_hat(v) >= c_best and self.f_hat(w) >= c_best
        )

        # Move disconnected vertices back to sample set
        self.x_samples.update(v for v in self.Tree.V if self.g_T(v) == float('inf'))

        # Keep only connected vertices in tree
        self.Tree.V = set(v for v in self.Tree.V if self.g_T(v) < float('inf'))


    def _enqueue_vertex(self, v):
        """Insert vertex v into the vertex heap with its key."""
        if v not in self.Tree.Qv:
            return
        key = self.f_hat(v)  # heuristic solution cost via v
        heapq.heappush(self._Qv_heap, (key, self._heap_counter, v))
        self._heap_counter += 1

    def _best_vertex_entry(self):
        """Peek best (key, v) from vertex heap, skipping stale entries."""
        while self._Qv_heap:
            key, _, v = self._Qv_heap[0]
            if v in self.Tree.Qv:
                return key, v
            # stale entry
            heapq.heappop(self._Qv_heap)
        return float('inf'), None

    def best_vertex(self):
        """Return best vertex in Qv (does not remove, expand_vertex will)."""
        _, v = self._best_vertex_entry()
        return v

    def pop_vertex(self):
        """Return key for best vertex without removing it, inf if none."""
        key, _ = self._best_vertex_entry()
        return key

    def _edge_key(self, v, x):
        return self.g_T(v) + self.c_hat(v, x) + self.h_hat(x)

    def _enqueue_edge(self, edge):
        if edge not in self.Tree.Qe:
            return
        v, x = edge
        key = self._edge_key(v, x)
        heapq.heappush(self._Qe_heap, (key, self._heap_counter, edge))
        self._heap_counter += 1

    def _best_edge_entry(self):
        while self._Qe_heap:
            key, _, edge = self._Qe_heap[0]
            if edge in self.Tree.Qe:
                return key, edge
            heapq.heappop(self._Qe_heap)
        return float('inf'), (None, None)

    def best_edge(self):
        _, edge = self._best_edge_entry()
        return edge

    def pop_edge(self):
        key, _ = self._best_edge_entry()
        return key
 




    def get_path(self):
        """
        Retrieve the path from start to goal by backtracking from the goal vertex.
        :param self: self is the bit star object
        """
        path = []
        current = self.Tree.goal
        if current.parent is None:
            print("goal node has no parent")
        while current is not None:
            path.append((current.x, current.y))
            current = current.parent

        path.reverse()
        return path


    
    def g_T(self, x):

        """
        cost-to-come value from start to x in the current tree
        :param self: self is the bit star object
        :param x: x is a verticie
        """
        if x == self.Tree.start:
            return 0.0
        elif x.parent is None:
            return float('inf')
        else:
            return self.g_T(x.parent) + self.calc_dist(x, x.parent) 



    def g_hat(self, x):

        """
        admissible cost-to-come value from start to x
        
        :param self: self is the bit star object
        :param x: x is a verticie
        """
        return self.calc_dist(self.Tree.start, x)

    def h_hat(self, x):
          """
          admissible cost-to-go value from x to goal
          :param self: self is the bit star object
          :param x: x is a verticie
          """
          return self.calc_dist(x, self.Tree.goal)
    
    def f_hat(self, v):
          """
          calculated the admissible estimate of the solution which passes through vertex v
          :param self: self is the bit star object
          :param v: v is a verticie
          """
          return self.g_hat(v) + self.h_hat(v) 
    
    def c_hat(self, v, w):
          
          """
          calculated the admissible estimate of the solution which is constrianed to pass through
          the edge between v anc w. v is the parent verticie and w is the child verticie.
          
          :param self: self is the bit star object
          :param v: v is a verticie 
          :param w: w is a verticie
          """

          return self.calc_dist(v, w)
    

    def c_actual(self, v, w):
            """
            calculate the actual cost between two vertices v and w
            :param self: self is the bit star object
            :param v: v is a verticie
            :param w: w is a verticie
            """

            if not self.collision_free(v, w):
                return float('inf')

            return self.calc_dist(v, w)

    def collision_free(self, v, w):
            """
            check if the path between two vertices v and w is collision-free
            :param self: self is the bit star object
            :param v: v is a verticie
            :param w: w is a verticie
            """
            # Placeholder for actual collision checking logic
            return True 

    def calc_dist(self,v ,w):
          """
          calculate the euclidean distance between two vertices v and w
          :param self: self is the bit star object
          :param v: v is a verticie
          :param w: w is a verticie
          """
          dx = v.x - w.x
          dy = v.y - w.y
          return ((dx**2) + (dy**2))**0.5
    
    def calc_radius(self, q):
        return 30.0


    

    def sample_free(self, n):
        """
        Very simple 2D uniform sampling in a fixed box.
        Replace with sampling from your actual free space / octomap.
        """
        samples = set()
        # TODO: tune these bounds to your RotorS world
        min_x, max_x = -100.0, 100.0
        min_y, max_y = -100.0, 100.0

        while len(samples) < n:
            x = random.uniform(min_x, max_x)
            y = random.uniform(min_y, max_y)
            v = Vertex(x, y)
            # Optional: discard if not collision_free(start, v) or in obstacle
            samples.add(v)

        return samples
    


if __name__ == "__main__":
    start = Vertex(5.0, 5.0)
    goal = Vertex(80.0, 80.0)
    tree = Tree(start, goal, radius=3.0)
    bit_star_planner = bit_star(tree, n_iters=100, sample_size=200)
    path = bit_star_planner.planning()
    print("Planned path:", path)
