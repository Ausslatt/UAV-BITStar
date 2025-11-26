class Environment2d(object):
	
	
	
	def __init__(self, xmin, xmax, ymin, ymax):
		self.bound = ((xmin,xmax),(ymin, ymax))
		
		
	def is_free(self, x,y):
		(xmin,xmax),(ymin,ymax) = self.bound
		
		if x < xmin or x > xmax:
			return False
			
		elif y < ymin or y > ymax:
			return False
			
		return True
		
		
	def line_is_free(self, p, q):
		return self.is_free(p[0],p[1]) and self.is_free(q[0], q[1])


