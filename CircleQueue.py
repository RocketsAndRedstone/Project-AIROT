class CircleQueue:

    #Declares needed parameters for the queue
    def __init__(self , maxSize):
        self.maxSize = maxSize
        self.head = 0
        self.numElements= 0
        self.circle = [None] * maxSize

    #returns the value of the current item in the queue without removing it
    def peek(self):
        if(self.head > self.maxSize):
            self.head = self.maxSize -1
        return self.circle[self.head - 1]
    
    #adds an atem to the queue
    def enqueue(self,  toAdd):
        if(self.head >= self.maxSize):
            self.head = 0
        
        self.circle[self.head] = toAdd
        self.numElements += 1
        self.head += 1
        
    #removes and returns the item currently denoted by the head
    def dequeue(self):
        if(self.head - 1 < self.numElements):
            self.head = self.maxSize -1
        self.head -= 1
        toReturn  = self.circle[self.head]
        self.circle[self.head] = None
        
        self.numElements -= 1

        return toReturn
    
    #emptys and resets the queue
    def clear(self):
        self.circle = []
        self.numElements = 0

    #returns the size of the queue
    def size(self):
        return self.numElements