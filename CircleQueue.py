class CircleQueue:

    def __init__(self , maxSize):
        self.maxSize = maxSize
        self.head = 0
        self.numElements= 0
        self.circle = [None] * maxSize

    def peek(self):
        if(self.head < self.numElements):
            self.head = self.maxSize -1
        return self.circle[self.head]
    
    def enqueue(self,  toAdd):
        if(self.head >= self.maxSize):
            self.head = 0
        
        self.circle[self.head] = toAdd
        self.numElements += 1
        self.head += 1
        

    def dequeue(self):
        if(self.head < self.numElements):
            self.head = self.maxSize -1
        toReturn  = self.circle[self.head]
        self.head -= 1
        self.numElements -= 1

        return toReturn
    
    def clear(self):
        self.circle = []
        self.numElements = 0

    def size(self):
        return self.numElements