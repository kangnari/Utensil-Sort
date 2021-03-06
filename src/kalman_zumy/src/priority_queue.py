"""Lifted directly from: https://teacode.wordpress.com/2013/08/02/algo-week-5-heap-and-dijkstras-shortest-path/"""
import scipy as sp
import numpy as np
import matplotlib.pyplot as plt
from scipy import signal as signal
from scipy import ndimage as ndimage
from numpy import random as random
import Queue
import heapq

class PriorityQueue(object):
    """Priority queue based on heap, capable of inserting a new node with
    desired priority, updating the priority of an existing node and deleting
    an abitrary node while keeping invariant"""
 
    def __init__(self, heap=[]):
        """if 'heap' is not empty, make sure it's heapified"""
 
        heapq.heapify(heap)
        self.heap = heap
        self.entry_finder = dict({i[-1]: i for i in heap})
        self.REMOVED = '<remove_marker>'
 
    def insert(self, nodeIn, priority=0):
        """'entry_finder' bookkeeps all valid entries, which are bonded in
        'heap'. Changing an entry in either leads to changes in both."""
        node = self.ptToString(nodeIn)
        if node in self.entry_finder:
            self.delete(node)
        entry = [priority, node]
        self.entry_finder[node] = entry
        heapq.heappush(self.heap, entry)
 
    def delete(self, node):
        """Instead of breaking invariant by direct removal of an entry, mark
        the entry as "REMOVED" in 'heap' and remove it from 'entry_finder'.
        Logic in 'pop()' properly takes care of the deleted nodes."""
 
        entry = self.entry_finder.pop(node)
        entry[-1] = self.REMOVED
        return entry[0]
 
    def pop(self):
        """Any popped node marked by "REMOVED" does not return, the deleted
        nodes might be popped or still in heap, either case is fine."""
 
        while self.heap:
            priority, node = heapq.heappop(self.heap)
            if node is not self.REMOVED:
                del self.entry_finder[node]
                return priority, self.stringToPt(node)
        raise KeyError('pop from an empty priority queue')
        
    def ptToString(self, pt):
        return str(pt[0]) + "," + str(pt[1])
    
    def stringToPt(self, inStr):
        return [int(x.strip()) for x in inStr.split(',')]