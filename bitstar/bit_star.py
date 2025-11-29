

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
	
	def __init__(self):
      
		self.start = start
		self.goal = goal
		self.radius = 3.0
		self.V = set()
		self.E = set()
		self.Qe = set()
		self.Qv = set()
		self.V_old = set()

class bit_star(object):

    def __init__(self, tree, start, goal, n_iters):
        """
        Initialize the BIT* algorithm with a tree, start and goal vertices, and number of iterations.
        :param tree: tree object containing vertices and edges
        :param start: start vertex
        :param goal: goal vertex
        :param n_iters: number of iterations to run the algorithm
        """
        self.Tree = tree
        self.start = start
        self.goal = goal
        self.n_iters = n_iters
        self.x_samples = set()
    
    
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
          return self.g_hat(self, v) + self.h_hat(self,v) 
    
    def c_hat(self, v, w):
          
          """
          calculated the admissible estimate of the solution which is constrianed to pass through
          the edge between v anc w. v is the parent verticie and w is the child verticie.
          
          :param self: self is the bit star object
          :param v: v is a verticie 
          :param w: w is a verticie
          """
          g_hat = self.g_hat(v)
          f_hat = self.f_hat(w)
          edge = self.calc_dist(v,w)

          return g_hat + edge + f_hat  

    

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


    def expand_vertex(self, v):
        """
        Expands vertex v by adding candidate edges to the edge queue Qe.
        :param self: self is the bit star object
        :param v: v is a verticie
        """
        g_goal = self.g_T(self.Tree.goal)

        # Remove v from the vertex queue
        self.Tree.Qv -= {v}

        # Nearby samples
        X_near = {
            x for x in self.x_samples
            if self.calc_dist(x, v) <= self.Tree.radius
        }

        # Candidate edges from v to nearby samples
        self.Tree.Qe.update({
            (v, x) for x in X_near
            if self.g_hat(v) + self.c_hat(v, x) + self.h_hat(x) < g_goal
        })

        # If v has not been expanded before, also consider nearby vertices
        if v not in self.Tree.V_old:
            V_near = {
                w for w in self.Tree.V
                if self.calc_dist(w, v) <= self.Tree.radius
            }

            self.Tree.Qe.update({
                (v, w) for w in V_near
                if (v, w) not in self.Tree.E
                and self.g_hat(v) + self.c_hat(v, w) + self.h_hat(w) < g_goal
                and self.g_T(v) + self.c_hat(v, w) < self.g_T(w)
            })

        def prune(self, c_best):
            """
            Prunes the tree and samples based on the current best solution cost c_best.
            :param self: self is the bit star object
            :param c_best: c_best is the cost of the best solution found so far
            """
            self.x_samples -= {
            x for x in self.x_samples if self.f_hat(x) >= c_best
            }

            self.Tree.V -= {
            v for v in self.Tree.V if self.f_hat(v) > c_best
            }

            self.Tree.E -= {
            (v, w) for (v, w) in self.Tree.E
            if self.f_hat(v) >= c_best and self.f_hat(w) >= c_best
            }

            self.x_samples.update({v for v in self.Tree.V if self.g_T(v) == float('inf')})

            self.Tree.V = {v for v in self.Tree.V if self.g_T(v) < float('inf') }

    
