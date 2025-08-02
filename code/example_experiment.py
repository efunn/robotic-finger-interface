from psychopy import core, event, visual
from keyboard import *

class ExampleExperiment(object):
    def __init__(self):
        event.globalKeys.add(key='q', func=self.quit)
        self.running = True
        self.win = visual.Window()
        self.kb = KeyboardWrapper('example-robot-config')

    def run_main_loop(self):
        while self.running:
            # any experiment logic here!
            print(self.kb.all_pos[:])
            # ...
            self.win.flip()

    def quit(self):
        self.running = False
        self.kb.shutdown()
        core.quit()

if __name__ == '__main__':
    experiment = ExampleExperiment()
    experiment.run_main_loop()
