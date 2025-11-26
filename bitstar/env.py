class Environment2d(object):
	
	
	
	def __init__(self, xmin, xmax, ymin, ymax):
		self.bound = ((xmin,xmax),(ymin, ymax))
		
		
	def is_free(self, x,y):
		if x > max or x < xmin:
			return False
			
		elif y > ymax or y < ymin:
			return False
		
		return True
		
		
	def line_is_free(self, p, q):
		return is_free(p[0],p[1]) and is_free(q[0], q[1])
