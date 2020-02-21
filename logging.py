import time

class logger:

    def __init__(self, plots, starttime):
        self.plots = plots
        open("logFile.txt", 'w').close() #opens file to erase content
        self.starttime = starttime

    def log(self, timestamp, x, y, z):
        
        with open("logFile.txt", "a") as file:
            file.write(str(timestamp-starttime) + "," + str(x) + "," + str(y) +","+ str(z) +"\n")
        print('[%d]: %d, %d, %d' % (timestamp-starttime, x, y, z))

if __name__ == '__main__':
    starttime = int(round(time.time() * 1000))
    lgz = logger(2, starttime)
    t1 = int(round(time.time() * 1000))
    lgz.log(t1, 1, 2, 3)
