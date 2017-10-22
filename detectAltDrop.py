from random import uniform
import threading
import time
import matplotlib.pyplot as plt
from sys import argv
from collections import deque

def main():
    alt = Altimeter()
    alt.start()
    
    SIZE = 10
    recentq = deque([], maxlen=SIZE)
    oldq = deque([], maxlen=SIZE)
    index = 0
    while(True):
        index += 1
        reading = alt.getAltitude()
        out = None
        len(recentq)
        if len(recentq) == SIZE:
            out = recentq.popleft()
        recentq.append(reading)
        if out is not None:
            oldq.append(out)
        if len(recentq) == len(oldq):
            sum_recent = sum(list(recentq))
            sum_old = sum(list(oldq))
            if sum_recent < sum_old:
                break
        time.sleep(uniform(0.005,0.015))
    if len(argv) > 1 and argv[1] == "-show":
        length = len(alt.vals)
        plt.plot([i for i in range(index)],alt.vals[:index], "bo")
        plt.plot([i for i in range(index,length)],alt.vals[index:], "ro")
        plt.show()


class Altimeter:
    def __init__(self):
        self.currentVal = 0
        self.vals = [self.genVal(i) for i in range(-280,280)]
        self.thread = None

    def getAltitude(self):
        return self.currentVal
    
    def modifyValue(self):
        for i,n in enumerate(self.vals):
            self.currentVal = n
            time.sleep(0.01)
        return

    def genVal(self,n):
        a = -1
        b = 0
        c = 80000
        x = float(n) + uniform(-3.0,3.0)
        val = (a*math.pow(x,2)) + (b*x) + c
        return val

    def start(self):
        self.thread = threading.Thread(target=self.modifyValue)
        self.thread.start()


if __name__ == "__main__":
    main()