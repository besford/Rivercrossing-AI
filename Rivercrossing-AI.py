import time
import heapq
from itertools import combinations

class RivercrossingProblem():
	def __init__(self, pSouth, side, pNorth, time):
		self.pSouth = pSouth
		self.side = side
		self.pNorth = pNorth
		self.time = time
		self.parent = None
	
	def isGoal(self):
		if (not self.pSouth) and self.pNorth:
			return True
		else:
			return False
	
	def isValid(self):
		return False

	def __eq__(self, other):
		return self.pSouth == other.pSouth \
					 and self.side == other.side \
					 and self.pNorth == other.pNorth \
					 and self.time == self.time 
	
	def __lt__(self, other):
		return self.time < other.time

	def __hash__(self):
		return hash((id(self.pSouth), self.side, id(self.pNorth), self.time))


def moveNorth(state, x, y):
	newSouth = list(state.pSouth)
	newNorth = list(state.pNorth)
	p1 = newSouth[x]
	p2 = newSouth[y]
	newSouth.remove(p1)
	newSouth.remove(p2)
	newTime = state.time + max(p1, p2)
	newNorth.append(p1)
	newNorth.append(p2)
	return RivercrossingProblem(sorted(newSouth), 'north', sorted(newNorth), newTime)


def moveOneNorth(state, x):
	newSouth = list(state.pSouth)
	newNorth = list(state.pNorth)
	p1 = newSouth[x]
	newSouth.remove(p1)
	newTime = state.time + p1
	newNorth.append(p1)
	return RivercrossingProblem(sorted(newSouth), 'north', sorted(newNorth), newTime)


def moveSouth(state, x, y):
	newSouth = list(state.pSouth)
	newNorth = list(state.pNorth)
	p1 = newNorth[x]
	p2 = newNorth[y]
	newNorth.remove(p1)
	newNorth.remove(p2)
	newTime = state.time + max(p1, p2)
	newSouth.append(p1)
	newSouth.append(p2)
	return RivercrossingProblem(sorted(newSouth), 'south', sorted(newNorth), newTime)


def moveOneSouth(state, x):
	newSouth = list(state.pSouth)
	newNorth = list(state.pNorth)
	p1 = newNorth[x]
	newNorth.remove(p1)
	newTime = state.time + p1
	newSouth.append(p1)
	return RivercrossingProblem(sorted(newSouth), 'south', sorted(newNorth), newTime)


def getSuccessors(state):
	children = []
	if state.side == 'south':
		if len(state.pSouth) > 1:
			for a, b in list(combinations(state.pSouth, 2)):
				newState = moveNorth(state, state.pSouth.index(a), state.pSouth.index(b))
				newState.parent = state
				children.append(newState)
		elif len(state.pSouth) == 1:
			newState = moveOneNorth(state, 0)
			newState.parent = state
			children.append(newState)
	elif state.side == 'north':
		for a in range(0, len(state.pNorth)):
			newState = moveOneSouth(state, a)
			newState.parent = state
			children.append(newState)
	return children
	
	
def bfs(problem):
	if problem.isGoal():
		return problem
	closed, fringe = set(), [problem]
	while fringe:
		state = fringe.pop(0)
		if state.isGoal():
			return state
		closed.add(state)
		children = getSuccessors(state)
		for child in children:
			if (child not in closed) or (child not in fringe):
				fringe.append(child)
	return None


def dfs(problem):
	if problem.isGoal():
		return problem
	closed, fringe = set(), [problem]
	while fringe:
		state = fringe.pop(0)
		if state.isGoal():
			return state
		closed.add(state)
		children = getSuccessors(state)
		for child in children:
			if (child not in closed) or (child not in fringe):
				fringe.insert(0, child)
	return None


import heapq
class PriorityQueue():
	def __init__(self):
		self.elems = []
	
	def put(self, elem, priority):
		heapq.heappush(self.elems, (priority, elem))
	
	def get(self):
		return heapq.heappop(self.elems)[1]

	def isEmpty(self):
		if len(self.elems) == 0:
			return True
		else:
			return False

def heuristic1(a, b):
	return len(a.pSouth) - 1

def heuristic2(a, b):
	return len(a.pSouth) / 2

def heuristic3(a, b):
	return (heuristic1(a, b) + heuristic2(a, b)) / 2

def aStar(problem, heuristic):
	if problem.isGoal():
		return problem
	
	fringe = PriorityQueue()
	origin, cost = {}, {}
	fringe.put(problem, 0)
	origin[problem] = None
	cost[problem] = 0
	
	while fringe:
		state = fringe.get()
		
		if state.isGoal():
			return state
		
		children = getSuccessors(state)
		for child in children:
			newCost = child.time - cost[state]
			if child not in cost or newCost < cost[child]:
				cost[child] = newCost
				priority = newCost + heuristic(state, child)
				fringe.put(child, priority)
				origin[child] = state
				
	return problem
	
	
def printSolution(solution):
	path = [solution]
	parent = solution.parent
	while parent:
		path.append(parent)
		parent = parent.parent

	for t in range(len(path)):
		state = path[len(path) - t - 1]
		print("  Move " + str(t + 1) + ": ")
		print("    South: " + str(state.pSouth))
		print("    North: " + str(state.pNorth))
		print("    Side: " + str(state.side))
		print("    Time: " + str(state.time) + " mins")
		print("")
		
def main():
	persons = []
	
	n = int(input("Enter number of persons: "))
	
	while len(persons) != n:
		userInput = input("Enter "+ str(n) +" persons: ")
		persons = list(map(int, userInput.split()))
		if len(persons) != n:
			print("Incorrect input")

	sTime = time.time()
	print("[===============================BFS Solution===============================]")
	Problem = RivercrossingProblem(persons, 'south', [], 0)
	solution = bfs(Problem)
	printSolution(solution)
	eTime = time.time()
	print("Total Time: approx. " + str(eTime - sTime) + " sec")

	print("")
	print("")

	sTime = time.time()
	print("[===============================DFS Solution===============================]")
	Problem2 = RivercrossingProblem(persons, 'south', [], 0)
	solution2 = dfs(Problem2)
	printSolution(solution2)
	eTime = time.time()
	print("Run Time: approx. " + str(eTime - sTime) + " sec")

	print("")
	print("")

	
	sTime = time.time()
	print("[===============================A* Solution===============================]")
	Problem3 = RivercrossingProblem(persons, 'south', [], 0)
	solution3 = aStar(Problem3, heuristic1)
	printSolution(solution3)
	eTime = time.time()
	print("Run Time: approx. " + str(eTime - sTime) + " sec")

	sTime = time.time()
	print("[===============================A* Solution2===============================]")
	Problem3 = RivercrossingProblem(persons, 'south', [], 0)
	solution3 = aStar(Problem3, heuristic2)
	printSolution(solution3)
	eTime = time.time()
	print("Run Time: approx. " + str(eTime - sTime) + " sec")

	sTime = time.time()
	print("[===============================A* Solution3===============================]")
	Problem3 = RivercrossingProblem(persons, 'south', [], 0)
	solution3 = aStar(Problem3, heuristic3)
	printSolution(solution3)
	eTime = time.time()
	print("Run Time: approx. " + str(eTime - sTime) + " sec")
	

		
if __name__ == "__main__":
	main()

