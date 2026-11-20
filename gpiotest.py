import Jetson.GPIO as G

G.setmode(G.BOARD)
G.setup(12, G.OUT, initial=G.LOW)

G.output(12, G.HIGH)
input ("Measure pin 12 now. Press Enter when done.")

G.output(12,G.LOW)
input("Measure pin 12 again. Press enter when done.")

G.cleanup()
