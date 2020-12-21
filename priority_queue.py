class Priority:

    def __init__(self, k1, k2):
        self.k1 = k1
        self.k2 = k2

    def __lt__(self, other):
        return self.k1 < other.k1 or (self.k1 == other.k1 and self.k2 < other.k2)

    def __le__(self, other):
        return self.k1 < other.k1 or (self.k1 == other.k1 and self.k2 <= other.k2)


class PriorityNode:

    def __init__(self, priority, vertex):
        self.priority = priority
        self.vertex = vertex

    def __le__(self, other):
        return self.priority <= other.priority

    def __lt__(self, other):
        return self.priority < other.priority


class PriorityQueue:
    def __init__(self):
        self.heap = []
        self.vertices_in_heap = []

    def top(self):
        return self.heap[0].vertex

    def top_key(self):
        if len(self.heap) == 0: return Priority(float('inf'), float('inf'))
        return self.heap[0].priority

    def pop(self):
        lastelt = self.heap.pop()  # raises appropriate IndexError if heap is empty
        self.vertices_in_heap.remove(lastelt)
        if self.heap:
            returnitem = self.heap[0]
            self.heap[0] = lastelt
            self._siftup(0)
        else:
            returnitem = lastelt
        return returnitem

    def insert(self, vertex, priority):
        item = PriorityNode(priority, vertex)
        self.vertices_in_heap.append(vertex)
        self.heap.append(item)
        self._siftdown(0, len(self.heap) - 1)

    def remove(self, vertex):
        self.vertices_in_heap.remove(vertex)
        for index, priority_node in enumerate(self.heap):
            if priority_node.vertex == vertex:
                self.heap[index] = self.heap[len(self.heap) - 1]
                self.heap.remove(self.heap[len(self.heap) - 1])
                break
        self.build_heap()

    def update(self, vertex, priority):
        for index, priority_node in enumerate(self.heap):
            if priority_node.vertex == vertex:
                self.heap[index].priority = priority
                break
        self.build_heap()

    def build_heap(self):
        n = len(self.heap)
        for i in reversed(range(n // 2)):
            self._siftup(i)

    def _siftdown(self, startpos, pos):
        newitem = self.heap[pos]
        while pos > startpos:
            parentpos = (pos - 1) >> 1
            parent = self.heap[parentpos]
            if newitem < parent:
                self.heap[pos] = parent
                pos = parentpos
                continue
            break
        self.heap[pos] = newitem

    def _siftup(self, pos):
        endpos = len(self.heap)
        startpos = pos
        newitem = self.heap[pos]
        childpos = 2 * pos + 1
        while childpos < endpos:
            rightpos = childpos + 1
            if rightpos < endpos and not self.heap[childpos] < self.heap[rightpos]:
                childpos = rightpos
            self.heap[pos] = self.heap[childpos]
            pos = childpos
            childpos = 2 * pos + 1
        self.heap[pos] = newitem
        self._siftdown(startpos, pos)
